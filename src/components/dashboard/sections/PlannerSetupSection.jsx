import { ALGORITHM_OPTIONS, TASK_OPTIONS } from "../../../lib/routeAlgorithms";
import { POINT_KIND_OPTIONS } from "../../../lib/zonePlanner";

const cardCls =
  "rounded-2xl bg-white/95 backdrop-blur border border-stone-200 shadow-[0_18px_40px_rgba(15,23,42,0.06)] p-4";
const inputCls =
  "w-full rounded-xl border border-stone-300 bg-white px-3 py-2 text-sm text-stone-900 shadow-sm transition focus:border-sky-400 focus:outline-none focus:ring-2 focus:ring-sky-100";
const subtleButtonCls =
  "rounded-xl border border-stone-300 bg-stone-100 px-3 py-2 text-sm font-semibold text-stone-800 shadow-sm transition hover:bg-stone-200 hover:border-stone-400";

export default function PlannerSetupSection({
  activePointKind,
  activeZoneName,
  adjustedVisitCount,
  algorithmKey,
  chargeCount,
  onActivePointKindChange,
  onAlgorithmChange,
  onClearChargePoints,
  onClearLimitPoints,
  onClearVisitPoints,
  onRouteTaskChange,
  polygonCount,
  routeBlocked,
  routeTaskKey,
  status,
  visitCount,
  zoneCount,
}) {
  return (
    <>
      <div className={cardCls}>
        <div className="text-[11px] uppercase tracking-[0.22em] text-stone-500 mb-2">Карта маршрута</div>
        <h2 className="text-2xl font-bold leading-tight">Планировщик маршрута робота</h2>
        <p className="mt-2 text-sm text-stone-600">
          Добавляйте точки посещения, станции зарядки и ограничивающие зоны.
          Теперь расчёт учитывает покрытие пола, массу груза и заданную скорость.
        </p>
      </div>
      <div className={cardCls}>
        <div className="flex items-center justify-between gap-3">
          <h3 className="text-sm font-semibold">Легенда и обзор</h3>
          <span className={`inline-flex rounded-full px-2.5 py-1 text-[11px] font-semibold ${routeBlocked ? "bg-rose-100 text-rose-700" : "bg-teal-100 text-teal-700"}`}>
            {routeBlocked ? "маршрут задевает контур" : "маршрут свободен"}
          </span>
        </div>
        <div className="mt-3 grid grid-cols-1 gap-2 text-xs text-stone-700">
          <div className="flex items-center gap-2"><span className="inline-block h-3 w-3 rounded-full bg-rose-600" /><span>Точки посещения</span></div>
          <div className="flex items-center gap-2"><span className="inline-block h-3 w-3 rounded-full bg-amber-500" /><span>Станции зарядки</span></div>
          <div className="flex items-center gap-2"><span className="inline-block h-3 w-3 rotate-45 bg-blue-600" /><span>Ограничивающая зона</span></div>
          <div className="flex items-center gap-2"><span className="inline-block h-3 w-3 rounded-full bg-yellow-500" /><span>Автосдвинутая безопасная точка</span></div>
          <div className="flex items-center gap-2"><span className={`inline-block h-[3px] w-8 rounded-full ${routeBlocked ? "bg-rose-600" : "bg-teal-700"}`} /><span>Линия маршрута</span></div>
        </div>
        <div className="mt-4 grid grid-cols-2 gap-2 text-xs">
          {[
            ["Маршрутных точек", visitCount], ["Станций зарядки", chargeCount],
            ["Запретных зон", zoneCount], ["Готовых контуров", polygonCount],
          ].map(([label, value]) => (
            <div key={label} className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
              <div className="text-stone-500">{label}</div><div className="mt-1 text-base font-semibold text-stone-900">{value}</div>
            </div>
          ))}
          <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 col-span-2">
            <div className="text-stone-500">Точек с автосдвигом</div><div className="mt-1 text-base font-semibold text-stone-900">{adjustedVisitCount}</div>
          </div>
        </div>
        <div className="mt-3 rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 text-xs text-stone-700">
          Активная зона: <span className="font-semibold text-stone-900">{activeZoneName}</span>
        </div>
      </div>
      <div className={cardCls}>
        <div className="text-xs text-stone-600 mb-2">Режим добавления точки</div>
        <div className="grid grid-cols-2 gap-2">
          {POINT_KIND_OPTIONS.map((option) => {
            const active = activePointKind === option.key;
            return (
              <button key={option.key} onClick={() => onActivePointKindChange(option.key)}
                className={`rounded-2xl border px-3 py-3 text-left transition ${active ? `${option.border} ${option.bg} shadow-sm` : "border-stone-200 bg-white hover:bg-stone-50"}`}>
                <div className={`text-xs uppercase tracking-[0.2em] ${active ? option.text : "text-stone-400"}`}>{option.shortLabel}</div>
                <div className="mt-1 text-sm font-semibold">{option.label}</div>
              </button>
            );
          })}
        </div>
        <div className="mt-3 grid grid-cols-1 gap-2 text-xs">
          <button onClick={onClearVisitPoints} className={subtleButtonCls}>Очистить маршрутные</button>
          <button onClick={onClearChargePoints} className={subtleButtonCls}>Очистить зарядки</button>
          <button onClick={onClearLimitPoints} className={subtleButtonCls}>Очистить зоны</button>
        </div>
      </div>
      <div className={cardCls}>
        <div className="text-xs text-stone-600 mb-1">Задача маршрута</div>
        <select className={inputCls} value={routeTaskKey} onChange={(event) => onRouteTaskChange(event.target.value)}>
          {TASK_OPTIONS.map((task) => <option key={task.key} value={task.key}>{task.label}</option>)}
        </select>
        <div className="text-xs text-stone-600 mt-3 mb-1">Алгоритм</div>
        <select className={inputCls} value={algorithmKey} onChange={(event) => onAlgorithmChange(event.target.value)}>
          {ALGORITHM_OPTIONS.map((option) => <option key={option.key} value={option.key}>{option.label}</option>)}
        </select>
        {status && <div className={`mt-3 text-sm ${routeBlocked ? "text-rose-700" : "text-emerald-700"}`}>{status}</div>}
      </div>
    </>
  );
}
