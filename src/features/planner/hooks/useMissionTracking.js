import {useCallback,useEffect,useState} from "react";
import {getMission} from "../services/missionClient";
const terminal = new Set(["completed","failed","cancelled"]);
export const useMissionTracking = ({onStarted,fetchMission = getMission}) => {
  const [missionId,setMissionId] = useState(null);
  const [state,setState] = useState(null);
  const track = useCallback(ack => {
    if (!ack?.missionId) return;
    setState({missionId:ack.missionId,status:ack.status || "persisted"});
    setMissionId(ack.missionId);
  },[]);
  useEffect(() => {
    if (!missionId) return;
    const controller = new AbortController();
    let timer;
    let started = false;
    const poll = async () => {
      try {
        const result = await fetchMission(missionId,controller.signal);
        if (controller.signal.aborted) return;
        if (result.missionId !== missionId) throw new Error("Ответ относится к другой миссии.");
        setState(result);
        if (result.status === "running" && !started) {started=true;onStarted?.();}
        if (terminal.has(result.status)) return;
      } catch(error) {
        if (controller.signal.aborted) return;
        setState(previous => ({...previous,connectionError:error.message}));
      }
      timer = setTimeout(poll,1000);
    };
    poll();
    return () => {controller.abort();clearTimeout(timer);};
  },[missionId,fetchMission,onStarted]);
  return {state,track};
};
