import { lazy, Suspense } from "react";
import { Navigate, Route, Routes } from "react-router-dom";
import Home from "./pages/Home";

const Dashboard = lazy(() => import("./pages/Dashboard"));

export default function App() {
  return (
    <Routes>
      <Route path="/" element={<Home />} />
      <Route
        path="/dashboard"
        element={
          <Suspense fallback={null}>
            <Dashboard />
          </Suspense>
        }
      />
      <Route path="*" element={<Navigate to="/" replace />} />
    </Routes>
  );
}
