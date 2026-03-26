# Webots Integration

`GPO-main` теперь содержит рабочий контур для `KUKA youBot` в `Webots`.

## Что используется

- Мир: [webots/worlds/youbot_only.wbt](C:/Users/User/Desktop/GPO-main/webots/worlds/youbot_only.wbt)
- Контроллер: [webots/controllers/youbot_web/youbot_web.c](C:/Users/User/Desktop/GPO-main/webots/controllers/youbot_web/youbot_web.c)
- Сборка контроллера: [webots/controllers/youbot_web/build_youbot_web.bat](C:/Users/User/Desktop/GPO-main/webots/controllers/youbot_web/build_youbot_web.bat)
- Runtime state:
  - [web_state/route.csv](C:/Users/User/Desktop/GPO-main/web_state/route.csv)
  - [web_state/route.json](C:/Users/User/Desktop/GPO-main/web_state/route.json)
  - [web_state/robot_state.json](C:/Users/User/Desktop/GPO-main/web_state/robot_state.json)

## Как это связано с UI

1. `Dashboard.jsx` строит маршрут
2. UI отправляет маршрут в `ws://127.0.0.1:9002/ui`
3. `ws-bridge.cjs` сохраняет маршрут в `web_state/route.csv`
4. Контроллер `youbot_web` в `Webots` читает `route.csv` и ведет робота по точкам
5. Контроллер пишет телеметрию в `web_state/robot_state.json`
6. `ws-bridge.cjs` читает этот файл и рассылает телеметрию в UI через `ws://127.0.0.1:9001`

## Запуск

1. Собрать native solver:
```powershell
npm run native:build
```

2. Собрать контроллер Webots:
```powershell
cd webots\controllers\youbot_web
.\build_youbot_web.bat
```

3. Запустить bridge:
```powershell
cd C:\Users\User\Desktop\GPO-main
npm run bridge
```

4. Запустить UI:
```powershell
npm run dev
```

5. В `Webots` открыть мир:
```text
C:\Users\User\Desktop\GPO-main\webots\worlds\youbot_only.wbt
```

6. Нажать `Run`

После этого:
- расставляешь точки в UI
- строишь маршрут
- отправляешь маршрут
- `youBot` должен поехать
- позиция робота должна вернуться на карту в UI
