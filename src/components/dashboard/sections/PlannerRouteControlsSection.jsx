import {
  ROUTE_CLEARANCE_MARGIN,
  SAFE_POINT_MARGIN,
} from "../../../lib/zonePlanner";

const cardCls =
  "rounded-2xl bg-white/95 backdrop-blur border border-stone-200 shadow-[0_18px_40px_rgba(15,23,42,0.06)] p-4";
const inputCls =
  "w-full rounded-xl border border-stone-300 bg-white px-3 py-2 text-sm text-stone-900 shadow-sm transition focus:border-sky-400 focus:outline-none focus:ring-2 focus:ring-sky-100";
const neutralButtonCls =
  "rounded-xl border border-slate-300 bg-slate-800 px-3 py-2 text-sm font-semibold text-white shadow-sm transition hover:bg-slate-900";

export default function PlannerRouteControlsSection({
  algorithmFields,
  hasRoute,
  isOptimizing,
  onAddRandomObstacle,
  onAlgorithmParamChange,
  onClearAll,
  onOptimizeRoute,
  onSendRoute,
  routeLength,
  selectedAlgorithmParams,
}) {
  return (
    <>
      <div className={cardCls}>
        <h3 className="text-sm font-semibold mb-3">Параметры алгоритма</h3>
        <div className="space-y-3">
          {algorithmFields.map((field) => (
            <label key={field.key}>
              <div className="text-xs text-stone-600 mb-1">{field.label}</div>
              <input type="number" min={field.min} max={field.max} step={field.step}
                value={selectedAlgorithmParams[field.key]} className={inputCls}
                onChange={(event) => onAlgorithmParamChange(field, event.target.value)} />
            </label>
          ))}
        </div>
      </div>
      <div className={cardCls}>
        <div className="mb-3 rounded-xl border border-teal-200 bg-teal-50 px-3 py-2 text-xs text-teal-800">
          Маршрут строится с зазором {ROUTE_CLEARANCE_MARGIN.toFixed(2)} м от контура,
          а целевые точки держатся минимум на {SAFE_POINT_MARGIN.toFixed(2)} м от запретной зоны.
        </div>
        <button onClick={onOptimizeRoute} disabled={isOptimizing}
          className={`w-full h-11 rounded-xl text-white font-semibold transition ${isOptimizing ? "bg-orange-400 cursor-wait" : "bg-orange-600 hover:bg-orange-700"}`}>
          {isOptimizing ? "Строим маршрут..." : "Построить маршрут"}
        </button>
        <button onClick={onSendRoute} disabled={isOptimizing}
          className={`mt-2 w-full h-11 rounded-xl text-white font-semibold transition ${isOptimizing ? "bg-emerald-400 cursor-not-allowed" : "bg-emerald-600 hover:bg-emerald-700"}`}>
          Отправить маршрут
        </button>
        <button onClick={onAddRandomObstacle} disabled={isOptimizing}
          className={`mt-2 w-full h-11 rounded-xl text-white font-semibold transition ${isOptimizing ? "bg-sky-400 cursor-not-allowed" : "bg-sky-600 hover:bg-sky-700"}`}>
          Добавить случайное препятствие
        </button>
        <button onClick={onClearAll} className={`mt-2 w-full h-11 ${neutralButtonCls}`}>Очистить всё</button>
        {hasRoute && <p className="mt-3 text-sm">Длина маршрута: <b>{routeLength.toFixed(2)} м</b></p>}
      </div>
    </>
  );
}
