#pragma once
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "sdmmc_cmd.h"
#include "ff.h"
#include "blackbox.h"
#include "record.h"
#include "web.h"

// The black box: every frame's record, the flow's 80x60 reduced frame and a full-resolution crop of the tag (and every
// fullEvery-th frame the whole frame) go to /blackbox/NNNN.bin on the microSD card, a new file per boot (blackbox.h has
// the format, fw/esp32cam/blackbox.py reads it). The detector only copies an entry into a ring in PSRAM, and drops it
// (counting) when the ring is full; a low-priority core-0 task moves the ring to the card in staging-sized writes from
// DMA-capable internal RAM, the only memory the SDMMC driver writes from without a sector-at-a-time bounce buffer.
namespace blackbox {
enum CardState { cardNone, cardFailed, cardIdle, cardRecording, cardError };
const char *const cardStateNames[] = {"none", "failed", "idle", "recording", "error"};
volatile CardState cardState = cardNone;
sdmmc_card_t *card = nullptr;
char drive[3] = "0:";
Ring ring;
uint8_t *staging = nullptr;
// Writer-owned while a file is open; the detector queues entries only while active.
volatile bool active = false, wantRecording = true;
volatile uint16_t fullEvery = config::blackBoxFullEvery;
// This file: frames queued, frames dropped because the ring was full, bytes written; and the card's size and free space.
volatile uint32_t framesQueued = 0, framesDropped = 0, fileNumber = 0, ringLowest = 0, writeLongestUs = 0;
volatile uint64_t bytesWritten = 0, cardBytes = 0, freeBytes = 0;
constexpr char mountPoint[] = "/sdcard", directory[] = "/sdcard/blackbox";

// Mounts the card in 1-bit SD mode on GPIO14 CLK, 15 CMD and 2 D0, if there is one. The card samples DAT3 at its first
// command and enters SPI mode if it is low; DAT3 is GPIO13, the sync line, so it is driven high until the card is up.
// Call before the sync line idles low; without a card or with an unreadable one the camera runs without a black box.
// The steps of esp_vfs_fat_sdmmc_mount, which would also link the SPI-mode card driver into the nearly full IRAM.
esp_err_t mountCard() {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_err_t result = sdmmc_host_init();
    if (result != ESP_OK)
        return result;
    static sdmmc_card_t found;
    BYTE number = 0xff;
    FATFS *volume = nullptr;
    if ((result = sdmmc_host_init_slot(host.slot, &slot)) == ESP_OK && (result = sdmmc_card_init(&host, &found)) == ESP_OK &&
        (result = ff_diskio_get_drive(&number)) == ESP_OK) {
        ff_diskio_register_sdmmc(number, &found);
        drive[0] = char('0' + number);
        if ((result = esp_vfs_fat_register(mountPoint, drive, 3, &volume)) == ESP_OK) {
            if (f_mount(volume, drive, 1) == FR_OK) {
                card = &found;
                return ESP_OK;
            }
            result = ESP_FAIL;
            esp_vfs_fat_unregister_path(mountPoint);
        }
        ff_diskio_unregister(number);
    }
    sdmmc_host_deinit();
    return result;
}

void mount() {
    pinMode(config::syncPin, OUTPUT);
    digitalWrite(config::syncPin, HIGH);
    const esp_err_t result = mountCard();
    digitalWrite(config::syncPin, LOW);
    if (result != ESP_OK) {
        // No answer at all is no card; anything else is a card that would not initialize or mount.
        cardState = result == ESP_ERR_TIMEOUT ? cardNone : cardFailed;
        return;
    }
    ring.size = config::blackBoxRingBytes;
    ring.bytes = static_cast<uint8_t *>(heap_caps_malloc(ring.size, MALLOC_CAP_SPIRAM));
    staging = static_cast<uint8_t *>(heap_caps_malloc(config::blackBoxStagingBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!ring.bytes || !staging) {
        cardState = cardFailed;
        card = nullptr;
        return;
    }
    cardBytes = uint64_t(card->csd.capacity) * card->csd.sector_size;
    ringLowest = ring.size;
    cardState = cardIdle;
}

// Opens the next blackbox/NNNN.bin, one past the highest there.
int openNext() {
    mkdir(directory, 0777);
    unsigned highest = 0;
    if (DIR *listing = opendir(directory)) {
        while (dirent *item = readdir(listing)) {
            unsigned number;
            char tail[8];
            if (sscanf(item->d_name, "%u.%7s", &number, tail) == 2 && !strcasecmp(tail, "bin") && number > highest)
                highest = number;
        }
        closedir(listing);
    }
    char path[48];
    fileNumber = highest + 1;
    snprintf(path, sizeof(path), "%s/%04u.bin", directory, unsigned(fileNumber));
    return open(path, O_WRONLY | O_CREAT | O_TRUNC);
}

void updateFree() {
    FATFS *volume;
    DWORD clusters;
    if (f_getfree(drive, &clusters, &volume) == FR_OK)
        freeBytes = uint64_t(clusters) * volume->csize * card->csd.sector_size;
}

// Core 0, priority 1: below the camera driver, Wi-Fi, the flow and the preview copier. Writes only whole staging
// buffers, so every write is sector-aligned and goes to the card in one multi-block command; the last partial one is
// written when the file closes. The file's size reaches the directory entry at each sync, once a second, so a power
// cut loses at most that.
void writerLoop(void *) {
    const uint32_t chunk = config::blackBoxStagingBytes;
    updateFree();
    int file = -1;
    uint32_t fill = 0;
    uint64_t fileBytes = 0;
    TickType_t synced = 0;
    for (;;) {
        if (file < 0) {
            if (!wantRecording || cardState == cardError) {
                if (cardState != cardError)
                    cardState = cardIdle;
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            file = openNext();
            if (file < 0) {
                cardState = cardError;
                continue;
            }
            FileHeader header;
            header.file = fileNumber;
            header.recordBytes = sizeof(FrameRecord);
            header.reducedWidth = flow::width;
            header.reducedHeight = flow::height;
            header.cropSize = config::blackBoxCropSize;
            header.fullWidth = config::outW;
            header.fullHeight = config::outH;
            header.fullEvery = fullEvery;
            header.startUs = uint32_t(esp_timer_get_time());
            header.fx = config::fx;
            header.fy = config::fy;
            header.cx = config::cx;
            header.cy = config::cy;
            memcpy(header.distortion, config::distortion, sizeof(header.distortion));
            header.tagSizeM = config::tagSizeM;
            header.tagId = config::tagId;
            strncpy(header.build, __DATE__ " " __TIME__, sizeof(header.build) - 1);
            memcpy(staging, &header, sizeof(header));
            fill = sizeof(header);
            fileBytes = 0;
            framesQueued = framesDropped = 0;
            bytesWritten = 0;
            ring.tail = ring.head;
            synced = xTaskGetTickCount();
            active = true;
            cardState = cardRecording;
        }
        const bool closing = !wantRecording || fileBytes + chunk > config::blackBoxFileBytes;
        if (closing && active) {
            // An entry the detector started before this sees active cleared is committed well within a frame.
            active = false;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        fill += ring.take(staging + fill, chunk - fill);
        if (fill == chunk || (closing && fill)) {
            const uint32_t started = uint32_t(esp_timer_get_time());
            const bool written = write(file, staging, fill) == int(fill);
            const uint32_t took = uint32_t(esp_timer_get_time()) - started;
            if (took > writeLongestUs)
                writeLongestUs = took;
            fileBytes += fill;
            bytesWritten = fileBytes;
            fill = 0;
            if (!written) {
                active = false;
                close(file);
                file = -1;
                cardState = cardError;
            }
            continue;
        }
        if (closing) {
            close(file);
            file = -1;
            updateFree();
            continue;
        }
        if (xTaskGetTickCount() - synced >= pdMS_TO_TICKS(1000)) {
            fsync(file);
            synced = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Core 1, once per frame while a file is open: queues the entry or counts it dropped, never waits. crop is null when
// the tag was not found; full is the frame buffer when this frame should carry the whole image.
inline void addFrame(const FrameRecord &record, const uint8_t *reduced, const CropSource *crop, const float corners[4][2],
                     const uint8_t *full) {
    if (!active)
        return;
    CropHeader place;
    if (crop)
        place = placeCrop(corners, config::blackBoxCropSize, crop->width, crop->height);
    if (encodeFrame(ring, framesDropped, &record, sizeof(record), reduced, flow::width * flow::height, crop, place, full,
                    config::outW * config::outH))
        framesQueued = framesQueued + 1;
    else
        framesDropped = framesDropped + 1;
    const uint32_t space = ring.space();
    if (space < ringLowest)
        ringLowest = space;
}

bool validName(const String &name) {
    if (name.length() < 5 || name.length() > 12 || !name.endsWith(".bin"))
        return false;
    for (size_t i = 0; i + 4 < name.length(); ++i)
        if (!isdigit(name[i]))
            return false;
    return true;
}

// /blackbox lists the files, /blackbox/file?name=NNNN.bin downloads one, and /blackbox/control?record=0|1&full_every=N
// stops (closing the file) or starts a new file and sets how often a frame carries the whole image. All of it runs in
// the core-0 web task; reading the card while recording competes with the writer, so download after landing.
void startServer() {
    web.on("/blackbox", HTTP_GET, []() {
        String body = "{\"state\":\"";
        body += cardStateNames[cardState];
        body += "\",\"file\":" + String(unsigned(fileNumber)) + ",\"files\":[";
        if (card) {
            if (DIR *listing = opendir(directory)) {
                bool first = true;
                while (dirent *item = readdir(listing)) {
                    if (!validName(item->d_name))
                        continue;
                    struct stat info;
                    const String path = String(directory) + "/" + item->d_name;
                    if (stat(path.c_str(), &info) != 0)
                        continue;
                    body += String(first ? "" : ",") + "{\"name\":\"" + item->d_name + "\",\"bytes\":" + String(unsigned(info.st_size)) + "}";
                    first = false;
                }
                closedir(listing);
            }
        }
        body += "]}";
        web.send(200, "application/json", body);
    });
    web.on("/blackbox/file", HTTP_GET, []() {
        const String name = web.arg("name");
        const String path = String(directory) + "/" + name;
        struct stat info;
        const int file = card && validName(name) && stat(path.c_str(), &info) == 0 ? open(path.c_str(), O_RDONLY) : -1;
        if (file < 0) {
            web.send(404, "text/plain", "no such file");
            return;
        }
        constexpr size_t pieceBytes = 8192;
        uint8_t *piece = static_cast<uint8_t *>(heap_caps_malloc(pieceBytes, MALLOC_CAP_SPIRAM));
        if (!piece) {
            close(file);
            web.send(500, "text/plain", "no memory");
            return;
        }
        web.setContentLength(info.st_size);
        web.sendHeader("Content-Disposition", "attachment; filename=" + name);
        web.send(200, "application/octet-stream", "");
        for (size_t left = info.st_size; left;) {
            const int n = read(file, piece, left < pieceBytes ? left : pieceBytes);
            if (n <= 0 || web.client().write(piece, n) != size_t(n))
                break;
            left -= n;
        }
        free(piece);
        close(file);
    });
    web.on("/blackbox/control", HTTP_GET, []() {
        if (web.hasArg("full_every"))
            fullEvery = uint16_t(max(0L, min(10000L, web.arg("full_every").toInt())));
        if (web.hasArg("record")) {
            wantRecording = web.arg("record").toInt() != 0;
            if (wantRecording && cardState == cardError)
                cardState = cardIdle;
        }
        web.send(200, "application/json", "{\"record\":" + String(wantRecording ? "true" : "false") + ",\"full_every\":" + String(unsigned(fullEvery)) + "}");
    });
}

// Starts the writer if a card is mounted; the web routes are there either way.
void start() {
    startServer();
    if (card)
        xTaskCreatePinnedToCore(writerLoop, "blackbox", 4096, nullptr, 1, nullptr, 0);
}

// The black box's part of /status.
int status(char *out, size_t size) {
    return snprintf(out, size,
                    "\"sd\":\"%s\",\"sd_file\":%u,\"sd_frames\":%u,\"sd_dropped\":%u,\"sd_bytes\":%llu,\"sd_card_mb\":%u,"
                    "\"sd_free_mb\":%u,\"sd_ring_lowest\":%u,\"sd_write_longest_us\":%u,\"sd_full_every\":%u",
                    cardStateNames[cardState], unsigned(fileNumber), unsigned(framesQueued), unsigned(framesDropped),
                    (unsigned long long)bytesWritten, unsigned(cardBytes >> 20), unsigned(freeBytes >> 20),
                    unsigned(ringLowest), unsigned(writeLongestUs), unsigned(fullEvery));
}
} // namespace blackbox
