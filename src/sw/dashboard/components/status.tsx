import type { LucideIcon } from 'lucide-react';
import type { ComponentProps } from 'react';

// A state in words beside its icon, in the color given: the header's readouts and the states beside panel headings.
export function Status({
  icon: Icon,
  iconClassName = '',
  className = '',
  children,
  ...props
}: ComponentProps<'span'> & { icon: LucideIcon; iconClassName?: string }) {
  return (
    <span
      className={`flex items-center gap-1.5 text-sm whitespace-nowrap ${className}`}
      {...props}
    >
      <Icon aria-hidden="true" className={`size-4 ${iconClassName}`} />
      {children}
    </span>
  );
}
