import { describe, expect, it } from "vitest";
import type { CalibrationProgress } from "@/lib/api";

describe("calibration progress contract", () => {
  it("recognizes a healthy running SLAM progress response", () => {
    const progress: CalibrationProgress = {
      found: true,
      step: "processing_frames",
      step_label: "SLAM обрабатывает кадры",
      frames_done: 25,
      frames_total: 100,
      slam_running: true,
      slam_crashed: false,
      elapsed: 12.4,
    };

    expect(progress.frames_done).toBeLessThanOrEqual(progress.frames_total!);
    expect(progress.slam_running).toBe(true);
    expect(progress.slam_crashed).toBe(false);
  });

  it("treats a crash as terminal even when the last frame is incomplete", () => {
    const progress: CalibrationProgress = {
      found: true,
      step: "error",
      step_label: "SLAM аварийно завершился",
      frames_done: 3,
      frames_total: 20,
      slam_running: false,
      slam_crashed: true,
      error: "SLAM завершился с кодом 139",
    };

    expect(progress.slam_crashed).toBe(true);
    expect(progress.slam_running).toBe(false);
    expect(progress.error).toContain("139");
  });
});