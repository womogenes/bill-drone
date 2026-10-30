// Reads the camera's multipart streams (app/api/camera): each part a JPEG and its headers, X-Tag on the preview, X-Time on FPV.
function indexOf(bytes: Uint8Array, pattern: number[], from = 0) {
  outer: for (let i = from; i <= bytes.length - pattern.length; i++) {
    for (let j = 0; j < pattern.length; j++)
      if (bytes[i + j] !== pattern[j]) continue outer;
    return i;
  }
  return -1;
}

export async function readParts(
  url: string,
  signal: AbortSignal,
  onFrame: (jpeg: Uint8Array, header: string) => void
) {
  const response = await fetch(url, { signal });
  if (!response.ok || !response.body)
    throw new Error((await response.json()).error ?? 'Preview unavailable.');
  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = new Uint8Array(0);
  for (;;) {
    const { value, done } = await reader.read();
    if (done) throw new Error('Preview ended.');
    const joined = new Uint8Array(buffer.length + value.length);
    joined.set(buffer);
    joined.set(value, buffer.length);
    buffer = joined;
    for (;;) {
      const headerEnd = indexOf(buffer, [13, 10, 13, 10]);
      if (headerEnd < 0) break;
      const header = decoder.decode(buffer.subarray(0, headerEnd));
      const length = Number(/Content-Length: *(\d+)/i.exec(header)?.[1] ?? 0);
      const start = headerEnd + 4;
      if (buffer.length < start + length + 2) break;
      if (length) onFrame(buffer.slice(start, start + length), header);
      buffer = buffer.slice(start + length + 2);
    }
  }
}
