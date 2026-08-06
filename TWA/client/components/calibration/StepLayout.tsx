import type { ReactNode } from "react";

/**
 * Standard full-screen centered card used by calibration steps.
 * Keeps the repeated `flex flex-col h-full` + `max-w-2xl` scaffolding in one place.
 */
export function StepLayout({ children }: { children: ReactNode }) {
  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 flex items-center justify-center p-6">
        <div className="max-w-2xl w-full space-y-6">{children}</div>
      </div>
    </div>
  );
}
