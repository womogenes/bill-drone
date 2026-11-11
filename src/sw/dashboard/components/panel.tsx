import { useId, type ReactNode } from 'react';

// One of a page's sections, with the padding and heading row they all share: the row is as tall as a button, so headings line up across columns whether or not theirs holds controls. header follows the heading in its row.
export function Panel({
  title,
  header,
  className = 'space-y-2',
  children,
}: {
  title: string;
  header?: ReactNode;
  className?: string;
  children?: ReactNode;
}) {
  const id = useId();
  return (
    <section aria-labelledby={id} className={`px-4 py-3 ${className}`}>
      <div className="flex h-8 items-center gap-2">
        <h2 id={id} className="font-semibold whitespace-nowrap">
          {title}
        </h2>
        {header}
      </div>
      {children}
    </section>
  );
}
