# Webots Integration

`GPO-main` теперь содержит рабочий контур для `KUKA youBot` в `Webots`.

## Что используется

- Мир: `webots/worlds/youbot_only.wbt`
- Контроллер: `webots/controllers/youbot_web/youbot_web.c`
- Платформенный ввод-вывод: `controller_io.c/.h`
- Mecanum-кинематика и slew limits колёс: `controller_drive.c/.h`
- Чистая математика лидара, контекста препятствий и уверенности трассы:
  `controller_lidar_math.c/.h`
- Локальное уклонение: lidar-классификация — `controller_avoidance_detection.c/.h`,
  выбор стороны и команда движения — `controller_avoidance_command.c/.h`, состояние,
  progress/stuck и detour-геометрия — `controller_avoidance_state.c/.h`.
- Чистый анализ RGB-кадра, camera obstacle observation и геометрия точек карты:
  `controller_camera.c/.h`
- Хранение и объединение camera obstacle/free-space ячеек:
  `controller_camera_map.c/.h`
- Чистые растровые примитивы виртуальной камеры:
  `controller_camera_render.c/.h`
- Чистые геометрические примитивы Mapping Survey:
  `controller_survey_geometry.c/.h`
- Состояние и повторяющиеся переходы Mapping Survey:
  `controller_survey_state.c/.h`
- Расписание периодических lifecycle-задач:
  `controller_lifecycle.c/.h`
- Тестируемый порядок выполнения одного controller step:
  `controller_step.c/.h`
- Математика навигации: `controller_math.c/.h`
- Загрузка и расчёт профиля движения: `controller_motion_profile.c/.h`
- Загрузка маршрута: `controller_route.c/.h`
- Чтение runtime-команд: `controller_runtime_command.c/.h`
- Сериализация телеметрии: `controller_telemetry.c/.h`
- Общие структуры и enum-типы контроллера: `controller_types.c/.h`
- Парсинг ограничивающих зон и покрытий: `controller_zones.c/.h`
- Сборка контроллера: `webots/controllers/youbot_web/build_youbot_web.bat`
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

3. Запустить bridge из корня проекта:
```powershell
cd C:\GPO\_ne_monolit
npm run bridge
```

4. Запустить UI:
```powershell
npm run dev
```

5. В `Webots` открыть мир:
```text
C:\GPO\_ne_monolit\webots\worlds\youbot_only.wbt
```

6. Нажать `Run`

После этого:
- расставляешь точки в UI
- строишь маршрут
- отправляешь маршрут
- `youBot` должен поехать
- позиция робота должна вернуться на карту в UI
