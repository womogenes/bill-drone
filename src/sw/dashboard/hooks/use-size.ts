import { useEffect, useState, type RefObject } from 'react';

// An element's size in pixels, followed as it changes, for a plot drawn at its real size (so its text keeps its size however wide the panel is); 400 x 300 until it is measured.
export function useSize(ref: RefObject<HTMLElement | null>) {
  const [size, setSize] = useState({ width: 400, height: 300 });
  useEffect(() => {
    const element = ref.current;
    if (!element) return;
    const resize = new ResizeObserver(() =>
      setSize({ width: element.clientWidth, height: element.clientHeight })
    );
    resize.observe(element);
    return () => resize.disconnect();
  }, [ref]);
  return size;
}
