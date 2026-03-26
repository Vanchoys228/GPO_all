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

Подробная схема модулей лежит в [PROJECT_SCHEME.md](./PROJECT_SCHEME.md).

## Что умеет проект

- Ставить точки посещения на координатной карте.
- Создавать несколько ограничивающих зон и замыкать их в полигоны.
- Автоматически выносить точки из запрещенной зоны к ближайшей безопасной позиции.
- Строить маршрут с учетом препятствий и зазоров безопасности.
- Считать порядок обхода через нативный C++ solver.
- Отправлять маршрут в `Webots` и получать телеметрию обратно в UI.

## Актуальная структура

```text
.
|-- src/
|   |-- components/dashboard/
|   |   |-- PlannerCanvas.jsx
|   |   |-- PlannerLeftSidebar.jsx
|   |   `-- PlannerRightSidebar.jsx
|   |-- lib/
|   |   |-- dashboardTelemetry.js
|   |   |-- plannerModel.js
|   |   |-- routeAlgorithms.js
|   |   |-- runtimeConfig.js
|   |   `-- zonePlanner.js
|   |-- pages/
|   |   `-- Dashboard.jsx
|   |-- App.jsx
|   `-- main.jsx
|-- native/
|   |-- apps/
|   |-- build/
|   |-- include/
|   `-- src/
|-- shared/
|   `-- coordinate-contract.json
|-- webots/
|   |-- controllers/youbot_web/
|   `-- worlds/youbot_only.wbt
|-- web_state/
|-- bridge-config.cjs
|-- ws-bridge.cjs
|-- telemetry-server.cjs
|-- COORDINATE_CONTRACT.md
`-- PROJECT_SCHEME.md
```

## Требования

- Node.js `22.12+` рекомендуется.
- npm `10+`.
- Visual Studio Build Tools / MSVC для сборки native solver и контроллера Webots.
- Установленный `Webots`, если нужен полный сценарий с роботом.

Сейчас проект у тебя собирается и на `Node 22.11.0`, но `Vite` предупреждает,
что лучше обновиться до `22.12+`.

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
C:\Users\User\Desktop\GPO-main\webots\worlds\youbot_only.wbt
```

Нажать `Run`, после чего:

1. поставить точки в UI;
2. при необходимости создать ограничивающие зоны;
3. нажать `Построить маршрут`;
4. нажать `Отправить маршрут`.

## Полезные команды

```powershell
npm run dev
npm run bridge
npm run telemetry:mock
npm run native:build
npm run webots:build
npm run lint
npm run build
npm run test
npm run test:bridge
```

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
- unit-тесты для нормализации телеметрии и модели планировщика.
- smoke-test bridge + solver HTTP API.

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

- `npm audit --omit=dev` сейчас чистый.
- Dev-аудит после обновления зависимостей тоже должен быть чистым или близким к этому, но основной критерий безопасности здесь — production runtime.
- Исторические страницы и старые UI-заглушки уже удалены, чтобы не путать поддержку.

## Следующие улучшения

- Автосохранение и загрузка карт/зон.
- История прогонов solver и сравнение алгоритмов.
- Отдельные интеграционные тесты для WebSocket-сценария UI -> bridge -> controller.
- Более строгий CI-пайплайн с Node `22.12+`.
