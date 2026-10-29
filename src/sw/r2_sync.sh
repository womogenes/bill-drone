#!/bin/sh
# Sync flight recordings with the shared R2 bucket: sw/r2_sync.sh pull|push
# Needs DRONE_R2_ACCESS_KEY_ID and DRONE_R2_SECRET_ACCESS_KEY in the environment.
cd "$(dirname "$0")"
export RCLONE_CONFIG_R2_TYPE=s3 RCLONE_CONFIG_R2_PROVIDER=Cloudflare RCLONE_CONFIG_R2_NO_CHECK_BUCKET=true \
  RCLONE_CONFIG_R2_ENDPOINT=https://6eb94b8b81a7c86e5686774958288204.r2.cloudflarestorage.com \
  RCLONE_CONFIG_R2_ACCESS_KEY_ID="$DRONE_R2_ACCESS_KEY_ID" RCLONE_CONFIG_R2_SECRET_ACCESS_KEY="$DRONE_R2_SECRET_ACCESS_KEY"
case "$1" in
  pull) rclone copy r2:bill-drone/recordings recordings ;;
  push) rclone copy recordings r2:bill-drone/recordings ;;
esac
