# Native Solver

Нативный слой внутри `GPO-main` считает порядок обхода точек на C++.

## Что здесь лежит

- `include/tsp` — общие заголовки и интерфейсы алгоритмов
- `src` — реализации `GA + Tabu`, `Simulated Annealing`, `Scatter Search`, `Cuckoo Search`
- `apps/gpo_route_solver.cpp` — CLI solver, который вызывает `ws-bridge.cjs`
- `build_msvc.bat` — сборка под MSVC

## Сборка

Из корня `GPO-main`:

```powershell
cd native
.\build_msvc.bat
```

После сборки должен появиться файл:

```text
native\build\gpo_route_solver.exe
```

## Как это используется

1. `Dashboard.jsx` отправляет точки, задачу и параметры на `http://127.0.0.1:9003/api/solve-route`
2. `ws-bridge.cjs` маппит UI-параметры на нативные параметры solver
3. `gpo_route_solver.exe` возвращает оптимальный порядок обхода
4. `Dashboard.jsx` поверх этого строит безопасный маршрут с обходом forbidden zones через `zonePlanner.js`

## Запуск полного контура

```powershell
npm install
npm run native:build
npm run bridge
npm run dev
```
