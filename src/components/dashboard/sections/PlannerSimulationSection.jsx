import {useEffect,useRef,useState} from "react";
import {connectSimulationStream} from "../../../features/planner/services/simulationStream";

function SimulationImage() {
  const [frame,setFrame]=useState(null);
  const [status,setStatus]=useState("connecting");
  const connection=useRef(null);
  useEffect(()=>{
    const endpoint=new URL("/simulation/",window.location.href);
    endpoint.protocol=endpoint.protocol === "https:" ? "wss:" : "ws:";
    const dispose=connectSimulationStream({url:endpoint.href,onFrame:setFrame,onStatus:setStatus});
    connection.current=dispose;
    return ()=>{connection.current=null;dispose();};
  },[]);
  return <>
    <p className="mb-2 text-xs text-slate-600" role="status">
      {status === "connected" ? "Прямой вид симуляции" : status === "reconnecting" ? "Симулятор недоступен. Повторное подключение…" : "Подключение к симулятору…"}
    </p>
    <div className="aspect-video overflow-hidden rounded-lg bg-slate-950">
      {frame && <img src={frame} alt="Робот в симуляторе Webots" className="h-full w-full object-contain" onError={()=>connection.current?.reconnect()}/>}
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
