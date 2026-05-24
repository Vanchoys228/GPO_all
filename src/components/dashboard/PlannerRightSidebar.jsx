import { DEFAULT_POINT_TASK, POINT_KIND_META, POINT_TASKS } from "../../lib/zonePlanner";
import CollapsibleSection from "./CollapsibleSection";

const zonePanelCardCls =
  "max-w-full min-w-0 rounded-[26px] border border-sky-200 bg-white p-4 shadow-[0_8px_28px_rgb(125_211_252/0.22)]";
const zoneCardBaseCls =
  "rounded-2xl border border-sky-100 p-3 bg-white shadow-[0_6px_18px_rgb(15_23_42/0.12)]";
const rowCls =
  "flex items-center justify-between gap-3 px-3 py-3 rounded-xl border border-stone-300 bg-white text-sm text-stone-900 shadow-sm hover:bg-stone-50 hover:border-stone-400 cursor-pointer transition";
const selectCls =
  "w-full rounded-xl border border-stone-300 bg-white px-3 py-2.5 text-sm text-stone-900 shadow-sm transition focus:border-sky-400 focus:outline-none focus:ring-2 focus:ring-sky-100";
const zonePrimaryButtonCls =
  "rounded-lg border border-sky-300 bg-sky-100 px-3 py-2 text-xs font-semibold text-sky-900 shadow-sm transition hover:bg-sky-200";
const zoneNeutralButtonCls =
  "rounded-lg border border-stone-300 bg-stone-100 px-3 py-2 text-xs font-semibold text-stone-900 shadow-sm transition hover:bg-stone-200";
const zoneDangerButtonCls =
  "rounded-lg border border-rose-300 bg-rose-50 px-3 py-2 text-xs font-semibold text-rose-800 shadow-sm transition hover:bg-rose-100";

const constraintChipCls =
  "inline-flex items-center rounded-full border border-stone-200 bg-stone-50 px-3 py-1.5 text-xs font-medium text-stone-900";

export default function PlannerRightSidebar({
  onCollapse,
  activeZone,
  activeZoneName,
  activeLimitZoneId,
  zoneEntries,
  visitEntries,
  chargeEntries,
  plannedVisitEntries,
  expandedPoint,
  hoveredPointIndex,
  visitsInsideLimitCount,
  polygonCount,
  adjustedVisitCount,
  routeBlocked,
  telemetry,
  telemetryWsUp,
  routeWsUp,
  solverApiUp,
  onCreateZone,
  onSelectZone,
  onToggleZoneClosed,
  onClearZone,
  onRemoveZone,
  onToggleExpandedPoint,
  onHoverPoint,
  onDeletePoint,
  onUpdatePointTask,
}) {
  const plannedVisitLookup = new Map(plannedVisitEntries.map((entry) => [entry.index, entry]));

  return (
    <aside className="flex h-full min-h-0 w-[330px] min-w-[330px] max-w-[330px] flex-none flex-col overflow-hidden border-l border-sky-100 bg-gradient-to-b from-sky-50 via-white to-cyan-50 xl:w-[350px] xl:min-w-[350px] xl:max-w-[350px]">
      <div className="flex shrink-0 items-start justify-between gap-2 border-b border-sky-200 bg-white px-3 py-2">
        <div className="min-w-0 pt-0.5">
          <div className="text-[11px] font-medium uppercase tracking-[0.16em] text-sky-800">
            Объекты карты
          </div>
          <div className="truncate text-base font-bold text-slate-900">Зоны и точки</div>
        </div>
        {onCollapse ? (
          <button
            type="button"
            title="Свернуть панель"
            aria-label="Свернуть правую панель"
            onClick={onCollapse}
            className="shrink-0 rounded-lg border border-sky-200 bg-white px-2 py-1.5 text-sky-700 shadow-sm transition hover:border-sky-300 hover:bg-sky-50"
          >
            <span className="text-lg leading-none" aria-hidden>
              ›
            </span>
          </button>
        ) : null}
      </div>

      <div className="min-h-0 min-w-0 max-w-full flex-1 overflow-x-hidden overflow-y-auto overscroll-contain p-4 space-y-4 [scrollbar-gutter:stable]">
        <div className={zonePanelCardCls}>
          <div className="flex items-start justify-between gap-3">
            <div>
              <div className="text-xs font-medium uppercase tracking-[0.14em] text-sky-800">
                Активная зона
              </div>
              <div className="mt-1 text-lg font-semibold text-slate-900 leading-snug">{activeZoneName}</div>
            </div>
            <div className="text-right text-sm text-slate-700">
              {activeZone ? (
                <>
                  <div>{activeZone.closed ? "Замкнута" : "Открыта"}</div>
                  <div className="mt-1">Точек: {activeZone.points.length}</div>
                </>
              ) : (
                <>
                  <div>—</div>
                  <div className="mt-1">Нет зоны</div>
                </>
              )}
            </div>
          </div>
          <div className="mt-3 text-sm text-slate-800 space-y-2 leading-relaxed">
            {activeZone ? (
              <>
                <p>В открытую зону можно добавлять и двигать точки кликом по карте.</p>
                <p>Замкнутая зона участвует в расчёте безопасного маршрута.</p>
              </>
            ) : (
              <p>
                Создайте зону кнопкой «Новая зона», затем выберите её в списке и добавьте точки контура на карте.
              </p>
            )}
          </div>
        </div>

        <div className={zonePanelCardCls}>
          <div className="flex items-center justify-between gap-3 mb-3">
            <div>
              <h3 className="text-base font-semibold text-slate-900">Ограничивающие зоны</h3>
              <div className="mt-1 text-sm text-slate-700">Контуры и статус</div>
            </div>
            <button
              type="button"
              onClick={onCreateZone}
              className="rounded-xl bg-sky-700 px-3 py-2.5 text-sm font-semibold text-white shadow-sm transition hover:bg-sky-800"
            >
              Новая зона
            </button>
          </div>
          <div className="space-y-2">
            {zoneEntries.length === 0 ? (
              <p className="rounded-2xl border border-dashed border-sky-200 bg-sky-50 px-4 py-4 text-sm leading-relaxed text-slate-800">
                Зон пока нет. Нажмите «Новая зона», выберите её в списке и отметьте точки контура на карте.
              </p>
            ) : (
              zoneEntries.map((zone) => {
              const active = zone.id === activeLimitZoneId;
              return (
                <div
                  key={zone.id}
                  className={`${zoneCardBaseCls} ${
                    active
                      ? "border-sky-300 bg-gradient-to-br from-sky-50 to-white"
                      : "border-sky-100 bg-white"
                  }`}
                >
                  <div className="flex items-start justify-between gap-3">
                    <button
                      type="button"
                      className="flex-1 min-w-0 rounded-xl px-2 py-1 text-left transition hover:bg-sky-50"
                      onClick={() => onSelectZone(zone.id)}
                    >
                      <div className="flex items-center gap-2">
                        <span className={`inline-block w-3 h-3 rounded-full ${zone.color.badge}`} />
                        <span className="text-base font-semibold text-slate-900">{zone.name}</span>
                        <span
                          className={`inline-flex rounded-full px-2 py-0.5 text-xs font-semibold ${
                            zone.closed ? "bg-emerald-100 text-emerald-700" : "bg-amber-100 text-amber-700"
                          }`}
                        >
                          {zone.closed ? "Замкнута" : "Открыта"}
                        </span>
                      </div>
                      <div className="mt-2 grid grid-cols-2 gap-2 text-sm text-slate-700">
                        <div>Точек: {zone.points.length}</div>
                        <div>{zone.points.length >= 3 ? "Контур готов" : "Нужно 3 точки"}</div>
                      </div>
                    </button>
                  </div>
                  <div className="mt-3 flex gap-2">
                    <button
                      type="button"
                      onClick={() => onToggleZoneClosed(zone.id)}
                      className={zonePrimaryButtonCls}
                    >
                      {zone.closed ? "Открыть" : "Замкнуть"}
                    </button>
                    <button type="button" onClick={() => onClearZone(zone.id)} className={zoneNeutralButtonCls}>
                      Очистить
                    </button>
                    <button type="button" onClick={() => onRemoveZone(zone.id)} className={zoneDangerButtonCls}>
                      Удалить
                    </button>
                  </div>
                </div>
              );
              })
            )}
          </div>
        </div>

        <CollapsibleSection
          sectionId="right.constraints"
          defaultOpen
          title="Контроль ограничений"
          description="Сводка по зонам и маршруту"
          className="border-sky-200 shadow-[0_6px_22px_rgb(125_211_252/0.22)]"
          headerClassName="rounded-[26px]"
        >
          <div className="flex flex-wrap gap-2">
            <span className={constraintChipCls}>В зонах: {visitsInsideLimitCount}</span>
            <span className={constraintChipCls}>Контуров: {polygonCount}</span>
            <span className={constraintChipCls}>Автосдвиг: {adjustedVisitCount}</span>
            <span
              className={`${constraintChipCls} ${
                routeBlocked ? "border-rose-200 bg-rose-50 text-rose-800" : ""
              }`}
            >
              Пересечение: {routeBlocked ? "да" : "нет"}
            </span>
          </div>
          <p className="mt-3 text-sm leading-relaxed text-stone-700">
            В обходе только замкнутые зоны. Точки перетаскиваются на карте. Если обход невозможен — отправка
            блокируется.
          </p>
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="right.visits"
          defaultOpen
          title="Точки посещения"
          description={`${visitEntries.length} шт.`}
          className="border-sky-200 shadow-[0_6px_22px_rgb(125_211_252/0.22)]"
          headerClassName="rounded-[26px]"
        >
          <div className="space-y-2">
            {visitEntries.map((entry) => {
              const expanded = expandedPoint === entry.index;
              const hovered = hoveredPointIndex === entry.index;
              const plannedEntry = plannedVisitLookup.get(entry.index);

              return (
                <div key={entry.index}>
                  <div
                    role="button"
                    tabIndex={0}
                    className={`${rowCls} ${hovered ? "border-sky-300 bg-sky-50 shadow-sm" : ""}`}
                    onClick={() => onToggleExpandedPoint(expanded ? null : entry.index)}
                    onKeyDown={(event) => {
                      if (event.key === "Enter" || event.key === " ") {
                        event.preventDefault();
                        onToggleExpandedPoint(expanded ? null : entry.index);
                      }
                    }}
                    onMouseEnter={() => onHoverPoint(entry.index)}
                    onMouseLeave={() => onHoverPoint(null)}
                  >
                    <div className="min-w-0">
                      <div className="text-sm font-semibold text-stone-900">
                        V{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
                      </div>
                      <div className="inline-flex mt-1 px-2 py-0.5 rounded-full border text-xs font-medium bg-rose-50 border-rose-200 text-rose-800">
                        {POINT_KIND_META.visit.label}
                      </div>
                      {plannedEntry?.adjusted ? (
                        <div className="inline-flex mt-1 ml-2 px-2 py-0.5 rounded-full border text-xs font-medium bg-amber-50 border-amber-200 text-amber-800">
                          Автосдвиг
                        </div>
                      ) : null}
                    </div>
                    <button
                      type="button"
                      title="Удалить точку"
                      aria-label={`Удалить точку посещения V${entry.order}`}
                      onClick={(event) => {
                        event.stopPropagation();
                        onDeletePoint(entry.index);
                      }}
                      className="flex items-center justify-center w-7 h-7 rounded-md border border-red-300 text-red-600 bg-red-50 hover:bg-red-600 hover:text-white transition"
                    >
                      ×
                    </button>
                  </div>
                  {expanded ? (
                    <div className="mt-2 ml-2 p-3 rounded-xl border border-stone-200 bg-white space-y-2">
                      <div className="text-sm text-stone-900">
                        x: {entry.point.x.toFixed(4)}, y: {entry.point.y.toFixed(4)}
                      </div>
                      {plannedEntry?.adjusted ? (
                        <div className="text-sm leading-relaxed text-amber-900">
                          Безопасная точка маршрута: x={plannedEntry.plannedPoint.x.toFixed(4)}, y=
                          {plannedEntry.plannedPoint.y.toFixed(4)}
                        </div>
                      ) : null}
                      <label>
                        <div className="text-sm font-medium text-stone-700 mb-1">Операция</div>
                        <select
                          className={selectCls}
                          value={entry.point.task || DEFAULT_POINT_TASK}
                          onChange={(event) => onUpdatePointTask(entry.index, event.target.value)}
                        >
                          {POINT_TASKS.map((task) => (
                            <option key={task} value={task}>
                              {task}
                            </option>
                          ))}
                        </select>
                      </label>
                    </div>
                  ) : null}
                </div>
              );
            })}
            {!visitEntries.length ? (
              <div className="text-sm leading-relaxed text-stone-800">
                Добавьте на карте точки, которые нужно посетить.
              </div>
            ) : null}
          </div>
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="right.charges"
          defaultOpen
          title="Станции зарядки"
          description={`${chargeEntries.length} шт.`}
          className="border-sky-200 shadow-[0_6px_22px_rgb(125_211_252/0.22)]"
          headerClassName="rounded-[26px]"
        >
          <div className="space-y-2">
            {chargeEntries.map((entry) => (
              <div
                key={entry.index}
                className={`${rowCls} ${
                  hoveredPointIndex === entry.index ? "border-amber-300 bg-amber-50 shadow-sm" : ""
                }`}
                onMouseEnter={() => onHoverPoint(entry.index)}
                onMouseLeave={() => onHoverPoint(null)}
              >
                <div className="min-w-0">
                  <div className="text-sm font-semibold text-stone-900">
                    C{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
                  </div>
                  <div className="inline-flex mt-1 px-2 py-0.5 rounded-full border text-xs font-medium bg-amber-50 border-amber-200 text-amber-900">
                    {POINT_KIND_META.charge.label}
                  </div>
                </div>
                <button
                  type="button"
                  title="Удалить станцию"
                  aria-label={`Удалить станцию зарядки C${entry.order}`}
                  onClick={(event) => {
                    event.stopPropagation();
                    onDeletePoint(entry.index);
                  }}
                  className="flex items-center justify-center w-7 h-7 rounded-md border border-red-300 text-red-600 bg-red-50 hover:bg-red-600 hover:text-white transition"
                >
                  ×
                </button>
              </div>
            ))}
            {!chargeEntries.length ? (
              <div className="text-sm leading-relaxed text-stone-800">
                Добавьте станции зарядки для длинных маршрутов.
              </div>
            ) : null}
          </div>
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="right.telemetry"
          defaultOpen={false}
          title="Телеметрия и связь"
          description="Позиция, сенсоры, состояние WebSocket"
          className="border-sky-200 shadow-[0_6px_22px_rgb(125_211_252/0.22)]"
          headerClassName="rounded-[26px]"
        >
          <div className="grid grid-cols-2 gap-2 text-sm text-stone-800">
            <div>x: {telemetry.x.toFixed(2)}</div>
            <div>y: {telemetry.y.toFixed(2)}</div>
            <div>z: {telemetry.z.toFixed(2)}</div>
            <div>yaw: {telemetry.yaw.toFixed(2)}</div>
            <div>lidar: {telemetry.perception?.lidar?.enabled ? "on" : "off"}</div>
            <div>hits: {telemetry.obstacleTrace?.length || 0}</div>
          </div>
          <div className="mt-3 space-y-2 text-sm text-stone-800 leading-relaxed">
            <div>
              WS Telemetry:{" "}
              <span className={telemetryWsUp ? "text-emerald-700 font-medium" : "text-red-600 font-medium"}>
                {telemetryWsUp ? "connected" : "disconnected"}
              </span>
            </div>
            <div>
              WS Route:{" "}
              <span className={routeWsUp ? "text-emerald-700 font-medium" : "text-red-600 font-medium"}>
                {routeWsUp ? "connected" : "disconnected"}
              </span>
            </div>
            <div>
              Native Solver:{" "}
              <span className={solverApiUp ? "text-emerald-700 font-medium" : "text-red-600 font-medium"}>
                {solverApiUp ? "connected" : "disconnected"}
              </span>
            </div>
          </div>
        </CollapsibleSection>
      </div>
    </aside>
  );
}
