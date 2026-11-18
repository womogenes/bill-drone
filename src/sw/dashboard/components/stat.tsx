import type { ReactNode } from 'react';

// A panel's readings, three to a row.
export function Stats({ children }: { children: ReactNode }) {
  return (
    <dl className="grid grid-cols-3 gap-x-4 gap-y-2 tabular-nums">
      {children}
    </dl>
  );
}

// One labeled reading in a panel's Stats; note is a second, quieter line under the value.
export function Stat({
  label,
  className,
  valueClassName,
  note,
  children,
}: {
  label: string;
  className?: string;
  valueClassName?: string;
  note?: ReactNode;
  children: ReactNode;
}) {
  return (
    <div className={className}>
      <dt className="text-sm text-muted-foreground">{label}</dt>
      <dd className={valueClassName}>{children}</dd>
      {note && <dd className="text-sm text-muted-foreground">{note}</dd>}
    </div>
  );
}
