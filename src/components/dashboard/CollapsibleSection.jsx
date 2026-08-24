import { useCallback, useState } from "react";
import { getSectionOpen, setSectionOpenInStorage } from "../../lib/plannerUiState";

export default function CollapsibleSection({
  sectionId,
  title,
  description,
  defaultOpen = true,
  children,
  className = "",
  contentClassName = "",
  headerClassName = "",
}) {
  const [open, setOpen] = useState(() => getSectionOpen(sectionId, defaultOpen));

  const toggle = useCallback(() => {
    setOpen((prev) => {
      const next = !prev;
      setSectionOpenInStorage(sectionId, next);
      return next;
    });
  }, [sectionId]);

  return (
    <div
      className={`max-w-full min-w-0 rounded-2xl border border-stone-200 bg-white shadow-[0_4px_16px_rgb(15_23_42/0.12)] ${className}`}
    >
      <button
        type="button"
        onClick={toggle}
        className={`flex w-full min-w-0 max-w-full items-start justify-between gap-3 rounded-2xl px-4 py-3 text-left text-stone-800 transition hover:bg-sky-50 ${headerClassName}`}
        aria-expanded={open}
      >
        <div className="min-w-0 flex-1">
          <h3 className="text-base font-semibold leading-snug text-stone-800">{title}</h3>
          {description ? (
            <p className="mt-1 text-sm leading-relaxed text-stone-600">{description}</p>
          ) : null}
        </div>
        <span
          className="mt-0.5 flex h-7 w-7 shrink-0 items-center justify-center rounded-lg border border-sky-200 bg-sky-50 text-sm font-semibold text-sky-700 select-none"
          aria-hidden
        >
          {open ? "-" : "+"}
        </span>
      </button>
      {open ? (
        <div className={`max-w-full min-w-0 border-t border-stone-100 px-4 pb-4 pt-3 ${contentClassName}`}>
          {children}
        </div>
      ) : null}
    </div>
  );
}
