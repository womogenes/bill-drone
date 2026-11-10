import type { ReactNode } from 'react';

// One labeled reading in a panel's <dl> grid; note is a second, quieter line under the value.
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
