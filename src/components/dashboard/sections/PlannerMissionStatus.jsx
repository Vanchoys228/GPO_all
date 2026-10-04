const labels = {prepared:"Подготовлена, отправка не подтверждена",persisted:"Сохранена, ожидается контроллер",accepted:"Принята контроллером",running:"Выполняется",holding_for_recovery:"Груз удерживается: требуется восстановление",cancelling:"Отмена запрошена, ожидается остановка",completed:"Завершена",failed:"Ошибка исполнения",cancelled:"Отменена"};
export default function PlannerMissionStatus({mission,onCancel,onResume}) {
  if (!mission) return null;
  const stages={approaching_object:"Подъезд к объекту",aligning:"Выравнивание",lowering_arm:"Опускание манипулятора",grasping:"Захват с пола",lifting:"Подъём",placing_on_platform:"Укладка на платформу",releasing_on_platform:"Освобождение захвата",returning_arm_for_transport:"Складывание руки",transporting:"Перевозка на платформе",picking_from_platform:"Подвод руки к платформе",grasping_from_platform:"Захват с платформы",lifting_from_platform:"Снятие с платформы",placing:"Размещение",releasing:"Отпускание",returning_arm:"Возврат манипулятора"};
  return <section className="rounded-xl border border-slate-200 bg-white p-3 text-sm" aria-label="Состояние миссии">
    <div className="font-semibold">{mission.operationType === "object_transfer" ? "Перенос объекта" : "Исполнение маршрута"}</div>
    <div role="status" className="mt-1">{labels[mission.status] || "Состояние неизвестно"}</div>
    {mission.operationType === "object_transfer" && <>
      <div className="mt-1 text-xs text-slate-600">{stages[mission.stage] || "Подготовка"} · {Math.round(mission.progress || 0)}%</div>
      <div className="mt-2 h-2 overflow-hidden rounded-full bg-slate-200"><div className="h-full bg-orange-500 transition-all" style={{width:`${Math.max(0,Math.min(100,mission.progress || 0))}%`}} /></div>
      {mission.attached && !mission.onPlatform && <div className="mt-1 text-xs font-medium text-emerald-700">Объект в захвате</div>}
      {mission.onPlatform && <div className="mt-1 text-xs font-medium text-blue-700">Объект находится на грузовой платформе</div>}
    </>}
    <div className="mt-1 break-all text-xs text-slate-500">ID: {mission.missionId}</div>
    {mission.connectionError && <div className="mt-1 text-amber-700">Нет связи с сервисом. Показано последнее известное состояние.</div>}
    {mission.errorCode && <div className="mt-1 text-xs text-red-700">Код ошибки: {mission.errorCode}</div>}
    {!mission.connectionError && mission.feedbackFresh === false && !["completed","cancelled","failed"].includes(mission.status) && <div className="mt-1 text-amber-700">Нет свежего подтверждения от контроллера.</div>}
    {onCancel && !["completed","cancelled","failed"].includes(mission.status) && <button type="button" className="mt-2 rounded border border-red-300 px-3 py-1 text-red-700 disabled:opacity-50" disabled={mission.status === "cancelling"} onClick={onCancel}>Отменить миссию</button>}
    {onResume && mission.status === "holding_for_recovery" && <button type="button" className="ml-2 mt-2 rounded border border-orange-400 bg-orange-50 px-3 py-1 text-orange-800" onClick={onResume}>Продолжить перенос</button>}
  </section>;
}
