# Запуск рабочей копии на Windows

## 1. Размещение

Распакуйте проект так, чтобы корневая папка находилась по адресу:

```text
C:\GPO\_ne\_monolit
```

В ней должны находиться `package.json`, `compose.yaml`, `ws-bridge.cjs`, папки
`bridge`, `native`, `src` и `webots`.

## 2. Проверка без Docker

Откройте PowerShell в папке проекта:

```powershell
cd C:\GPO\_ne\_monolit
npm install
npm run native:build
npm test
npm run lint
npm run build
```

Затем запустите bridge и frontend в разных окнах PowerShell:

```powershell
npm run bridge
```

```powershell
npm run dev
```

Webots пока запускается обычным способом с графическим окном. Откройте:

```text
C:\GPO\_ne\_monolit\webots\worlds\youbot_only.wbt
```

## 3. Проверка bridge в Docker

Убедитесь, что Docker Desktop запущен, затем выполните:

```powershell
cd C:\GPO\_ne\_monolit
docker compose build bridge
docker compose up bridge
```

Проверка HTTP:

```powershell
Invoke-RestMethod http://127.0.0.1:9003/health
```

Ожидается:

```text
ok              : True
solverAvailable : True
```

После этого запустите frontend локально:

```powershell
npm run dev
```

И откройте тот же мир Webots. Каталог `web_state` подключён в контейнер bridge,
поэтому существующий файловый обмен сохраняется.

## 4. Что проверять в Webots

1. Поставить минимум три точки.
2. Построить маршрут.
3. Отправить маршрут.
4. Убедиться, что `route.csv` обновился.
5. Запустить симуляцию Webots.
6. Проверить движение робота.
7. Проверить координаты и статус в интерфейсе.
8. Проверить лидар и камеру.

Если обычный режим работает, а Docker-режим нет, сохраните вывод:

```powershell
docker compose logs bridge
```

Не изменяйте одновременно контроллер Webots и форматы WebSocket-сообщений до
прохождения этой проверки.
