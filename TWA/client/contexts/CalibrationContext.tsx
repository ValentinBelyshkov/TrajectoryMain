import { createContext, useContext, type ReactNode } from "react";
import type { useProject } from "@/hooks/useProject";

export type CalibrationController = ReturnType<typeof useProject>;

const CalibrationContext = createContext<CalibrationController | null>(null);

export function CalibrationProvider({
  value,
  children,
}: {
  value: CalibrationController;
  children: ReactNode;
}) {
  return (
    <CalibrationContext.Provider value={value}>
      {children}
    </CalibrationContext.Provider>
  );
}

export function useCalibration() {
  const ctx = useContext(CalibrationContext);
  if (!ctx) {
    throw new Error("useCalibration must be used within a CalibrationProvider");
  }
  return ctx;
}
