# GPO

[![React 19](https://img.shields.io/badge/React-19-20232A?logo=react)](https://react.dev/)
[![Vite 7](https://img.shields.io/badge/Vite-7-646CFF?logo=vite&logoColor=white)](https://vite.dev/)
[![Webots](https://img.shields.io/badge/Webots-youBot-2563EB)](https://cyberbotics.com/)
[![Native Solver](https://img.shields.io/badge/C%2B%2B-TSP%20solver-0F172A)](https://isocpp.org/)

Интерактивный стенд для планирования маршрута мобильного робота `KUKA youBot`.
Пользователь ставит точки на карте, задает ограничивающие зоны, запускает
оптимизацию маршрута и отправляет готовый путь в `Webots`.

Проект объединяет:

- React-интерфейс для постановки точек и управления сценарием.
- Геометрический модуль для полигонов, безопасных отступов и обхода зон.
- Нативный C++ solver для задач маршрутизации.
- WebSocket bridge между UI, solver и контроллером робота.
- Контроллер `Webots` для `KUKA youBot`.

## Что умеет проект

- Ставить точки посещения на координатной карте.
- Создавать несколько ограничивающих зон и замыкать их в полигоны.
- Автоматически выносить точки из запрещенной зоны к ближайшей безопасной позиции.
- Строить маршрут с учетом препятствий и зазоров безопасности.
- Считать порядок обхода через нативный C++ solver.
- Отправлять маршрут в `Webots` и получать телеметрию обратно в UI.

## Актуальная структура

Границы модулей, сервисный слой, состояния миссий и отдельный запуск сервисов
описаны в [документации архитектуры](docs/MODULAR-ARCHITECTURE.md).

```text
.
|-- src/
|   |-- components/dashboard/
|   |   |-- PlannerCanvas.jsx
|   |   |-- PlannerLeftSidebar.jsx
|   |   |-- PlannerRightSidebar.jsx
|   |   `-- sections/
|   |-- features/planner/
|   |   |-- hooks/
|   |   |-- model/
|   |   `-- services/
|   |-- lib/
|   |   |-- dashboardTelemetry.js
|   |   |-- plannerModel.js
|   |   |-- routeAlgorithms.js
|   |   |-- runtimeConfig.js
|   |   |-- zonePlanner.js
|   |   |-- zonePlannerCoordinates.js
|   |   |-- zonePlannerGeometry.js
|   |   |-- zonePlannerPolygons.js
|   |   `-- zonePlannerRouting.js
|   |-- pages/
|   |   `-- Dashboard.jsx
|   |-- App.jsx
|   `-- main.jsx
|-- native/
|   |-- apps/
|   |-- include/
|   `-- src/
|-- bridge/
|   |-- artifacts/
|   |-- config/
|   |-- protocol/
|   |-- servers/
|   |-- solver/
|   `-- telemetry/
|-- shared/
|   `-- coordinate-contract.json
|-- webots/
|   |-- controllers/youbot_web/
|   `-- worlds/youbot_only.wbt
|-- web_state/
|   `-- .gitkeep
|-- bridge-config.cjs
|-- ws-bridge.cjs
|-- telemetry-server.cjs
`-- COORDINATE_CONTRACT.md
```

## Требования

- Node.js `22.19+` (хранилище использует встроенный `node:sqlite`).
- npm `10+`.
- Visual Studio Build Tools / MSVC для сборки native solver и контроллера Webots.
- Установленный `Webots`, если нужен полный сценарий с роботом.

## Настройка окружения

Скопируй пример переменных:

```powershell
Copy-Item .env.example .env
```

По умолчанию используются:

- `VITE_BRIDGE_HOST=127.0.0.1`
- `VITE_TELEMETRY_PORT=9001`
- `VITE_ROUTE_PORT=9002`
- `VITE_SOLVER_PORT=9003`
- `BRIDGE_HOST=127.0.0.1`
- `TELEMETRY_PORT=9001`
- `ROUTE_PORT=9002`
- `SOLVER_PORT=9003`

## Быстрый старт

### 1. Установить зависимости

```powershell
npm install
```

### 2. Собрать native solver

```powershell
npm run native:build
```

### 3. Собрать контроллер Webots

```powershell
npm run webots:build
```

### 4. Запустить bridge

В первом терминале:

```powershell
npm run bridge
```

Для независимого запуска вместо `npm run bridge` открой четыре терминала:

```powershell
npm run service:gateway
npm run service:planning
npm run service:route
npm run service:telemetry
```

Шлюз использует `WEB_STATE_DIR`, сервис миссий — отдельный `MISSION_STATE_DIR`
(по умолчанию `data/missions`). Не запускай общий bridge одновременно с отдельными
сервисами на тех же портах. Проверка готовности: `/ready` на портах 9001–9004;
порт 9004 относится к шлюзу Webots.

Резервное копирование, восстановление, секреты, ротация логов и остановка
описаны в [инструкции по эксплуатации](docs/OPERATIONS.md).

Сервисы читают `.env` из корня проекта. Относительные пути `WEB_STATE_DIR`,
`MISSION_STATE_DIR` и `SOLVER_PATH` также отсчитываются от корня проекта,
независимо от рабочего каталога процесса; абсолютные пути сохраняются.
GitHub Actions (`Service portability`) проверяет сборку Linux solver,
нативные тесты с включёнными assertions, тесты JavaScript и взаимодействие
четырёх сервисов из отдельных рабочих каталогов на Ubuntu.

При переходе со старой версии останови bridge и перенеси прежние записи миссий:

```powershell
New-Item -ItemType Directory -Force data/missions | Out-Null
# Выполнить, если в web_state/missions есть сохранённые миссии:
Copy-Item web_state/missions/*.json data/missions/
```

Сервис автоматически импортирует JSON в собственную SQLite-базу. Исходные JSON
сохраняются. Данные из незавершённых миссий будут восстановлены; при необходимости
отмени старую миссию в UI перед отправкой новой. Подробнее — в
[описании сервисов](docs/MODULAR-ARCHITECTURE.md).

### 5. Запустить frontend

Во втором терминале:

```powershell
npm run dev
```

Открыть в браузере:

```text
http://127.0.0.1:5173
```

### 6. Запустить Webots

Открыть мир:

```text
<корень-проекта>\webots\worlds\youbot_only.wbt
```

Нажать `Run`, после чего:

1. поставить точки в UI;
2. при необходимости создать ограничивающие зоны;
3. нажать `Построить маршрут`;
4. нажать `Отправить маршрут`;
5. следить за состоянием миссии; для замены активного маршрута нажать
   `Отменить миссию` и дождаться подтверждения остановки.

Редактирование сцены во время активной миссии не применяется к симулятору.
Для нестандартного `WEB_STATE_DIR` Webots должен быть запущен с тем же значением
переменной окружения, что и шлюз.

## Полезные команды

```powershell
npm run dev
npm run bridge
npm run telemetry:mock
npm run native:build
npm run native:test
npm run webots:build
npm run lint
npm run build
npm run test
npm run test:bridge
npm run test:services
npm run test:simulation
npm run test:webots
```

## Docker-стек

Compose запускает production frontend, WebSocket bridge и Linux-версию native
solver. Webots с графическим окном пока работает на Windows и использует общий
каталог `web_state`.

```bash
docker compose up --build
```

Headless Webots R2025a запускается отдельным профилем:

```bash
docker compose --profile simulation up --build
```

После запуска доступны прежние адреса:

- `ws://127.0.0.1:9001` — телеметрия;
- `ws://127.0.0.1:9002/ui` — маршруты и команды UI;
- `http://127.0.0.1:9003/health` — состояние bridge и solver.
- `http://127.0.0.1:8080` — frontend.

## Проверки качества

Перед изменениями и перед выдачей результата полезно прогонять:

```powershell
npm run lint
npm run build
npm run test
npm run test:bridge
```

Что покрыто сейчас:

- `lint` для frontend и node-скриптов.
- `build` production-сборки.
- unit- и компонентные тесты для planner model, hooks, services, геометрии,
  телеметрии и sidebar-секций.
- smoke-test bridge + solver HTTP API.
- автономные C++-тесты native protocol/problem/service и TSP-модулей через
  `npm run native:test`.
- автономные C-тесты модулей Webots-контроллера через `npm run test:webots`.

## Координатный контракт

Единый контракт координат описан в:

- [COORDINATE_CONTRACT.md](./COORDINATE_CONTRACT.md)
- [coordinate-contract.json](./shared/coordinate-contract.json)

Коротко:

- плоскость маршрута: `x/y`
- высота: `z`
- поза робота: `pose.{x,y,z,yaw}`
- CSV маршрута: `x,y,headingDeg`

## Что еще важно

- Проверяйте runtime- и dev-зависимости командой `npm audit` после каждого
  обновления lock-файла.
- Обновляйте зависимости отдельным изменением и повторяйте полный набор
  регрессионных проверок.
- Build artifacts (`dist`, `native/build`, Webots `.exe`) и runtime state не
  хранятся в Git и создаются локально командами сборки.

## Следующие улучшения

- Автосохранение и загрузка карт/зон.
- История прогонов solver и сравнение алгоритмов.
- Отдельные интеграционные тесты для WebSocket-сценария UI -> bridge -> controller.
- Более строгий CI-пайплайн с Node `22.12+`.
