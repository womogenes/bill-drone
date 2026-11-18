// Keys typed into a field or a dialog aren't flight commands.
export function typing(event: KeyboardEvent) {
  return (
    event.ctrlKey ||
    event.metaKey ||
    event.altKey ||
    (event.target instanceof HTMLElement &&
      !!event.target.closest(
        'input, textarea, select, [contenteditable="true"], [role="dialog"]'
      ))
  );
}
