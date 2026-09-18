import {useEffect,useRef,useState} from "react";
import {connectSimulationViewer} from "../../../features/planner/services/simulationViewer";

function SimulationImage() {
  const frame=useRef(null);
  const [status,setStatus]=useState("connecting");
  const [mode,setMode]=useState(null);
  const [pending,setPending]=useState(false);
  const [error,setError]=useState("");
  const connection=useRef(null);
  useEffect(()=>{
    const dispose=connectSimulationViewer({frame:frame.current,onStatus:setStatus,onMode:setMode,onError:setError});
    connection.current=dispose;
    return ()=>{connection.current=null;dispose();};
  },[]);
  const changeMode=async value=>{
    const active=connection.current;
    if (!active) return;
    setPending(true);setError("");
    try {await active.setMode(value);}
    catch (failure) {if (connection.current === active) setError(failure.message);}
    finally {if (connection.current === active) setPending(false);}
  };
  return <>
    <p className="mb-2 text-xs text-slate-600" role="status">
      {status === "connected" ? "Интерактивный вид симуляции" : status === "error" ? "Просмотрщик недоступен" : status === "reconnecting" ? "Симулятор недоступен. Повторное подключение…" : "Подключение к симулятору…"}
    </p>
    <div className="mb-2 flex gap-2" role="group" aria-label="Скорость симуляции">
      {[["realtime","Обычная"],["fast","Максимальная"]].map(([value,label])=><button
        key={value} type="button" aria-pressed={mode === value}
        disabled={status !== "connected" || pending}
        onClick={()=>changeMode(value)}
        title={value === "realtime" ? "Цель — 1× реального времени" : "Максимальная скорость, доступная компьютеру"}
        className={`rounded-md border px-2 py-1 text-xs disabled:opacity-50 ${mode === value ? "border-sky-600 bg-sky-600 text-white" : "border-slate-200 text-slate-700 hover:bg-sky-50"}`}
      >{label}</button>)}
    </div>
    {pending && <p className="mb-2 text-xs text-slate-600" role="status">Изменяем скорость…</p>}
    {error && <p className="mb-2 text-xs text-red-700" role="alert">{error}</p>}
    <div className="aspect-video overflow-hidden rounded-lg bg-slate-950">
      <iframe ref={frame} title="Робот в симуляторе Webots" className="h-full w-full border-0" allow="fullscreen"/>
    </div>
  </>;
}

export default function PlannerSimulationSection() {
  const [open,setOpen]=useState(false);
  return <section className="rounded-[18px] border border-sky-100 bg-white p-4">
    <button type="button" aria-expanded={open} onClick={()=>setOpen(value=>!value)} className="w-full text-left text-sm font-semibold">
      {open ? "Скрыть симуляцию" : "Показать симуляцию"}
    </button>
    {open && <div className="mt-3"><SimulationImage/></div>}
  </section>;
}
