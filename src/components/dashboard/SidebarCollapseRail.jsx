export default function SidebarCollapseRail({ side, onExpand, label }) {
  const isLeft = side === "left";
  const title = label || (isLeft ? "Открыть панель планирования" : "Открыть панель объектов");

  return (
    <div
      className={`flex h-full w-11 min-w-[2.75rem] max-w-[2.75rem] flex-none flex-col items-center overflow-hidden border-stone-200 bg-gradient-to-b from-stone-100 via-white to-slate-100 py-3 ring-1 ring-inset ring-stone-300 ${
        isLeft ? "border-r" : "order-last border-l"
      }`}
    >
      <button
        type="button"
        title={title}
        aria-label={title}
        onClick={onExpand}
        className="flex h-10 w-9 items-center justify-center rounded-xl border border-stone-300 bg-white text-stone-600 shadow-sm transition hover:border-sky-400 hover:bg-sky-50 hover:text-sky-800"
      >
        <span className="text-lg leading-none" aria-hidden>
          {isLeft ? "›" : "‹"}
        </span>
      </button>
    </div>
  );
}
