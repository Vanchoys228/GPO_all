import React from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it, vi } from "vitest";
import PlannerImportSection from "./PlannerImportSection";

describe("PlannerImportSection", () => {
  it("renders the supported graph file contract", () => {
    const html = renderToStaticMarkup(<PlannerImportSection onImportFile={vi.fn()} />);
    expect(html).toContain("Импорт графа");
    expect(html).toContain("Загрузить граф");
    expect(html).toContain(".json,.xlsx,.xls,.csv,application/json");
  });
});
