import { describe, expect, it, vi, beforeEach, afterEach } from "vitest";
import {
  controlTerraSLAMComponent,
  getCalibrationProgress,
  getCalibrationSession,
  getTerraSLAMStatus,
  processCalibrationSession,
} from "./api";

describe("TerraSLAM API client", () => {
  const fetchMock = vi.fn();

  beforeEach(() => {
    vi.stubGlobal("fetch", fetchMock);
    fetchMock.mockReset();
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("requests component status from the backend", async () => {
    fetchMock.mockResolvedValue(
      new Response(JSON.stringify({ components: [{ name: "slam", level: 0 }] }), {
        status: 200,
        headers: { "Content-Type": "application/json" },
      }),
    );

    const result = await getTerraSLAMStatus();

    expect(result).toEqual({ components: [{ name: "slam", level: 0 }] });
    expect(fetchMock).toHaveBeenCalledWith(
      "/api/control/terraslam/status",
      expect.objectContaining({ headers: expect.objectContaining({ "Content-Type": "application/json" }) }),
    );
  });

  it("sends component actions and preserves backend errors", async () => {
    fetchMock.mockResolvedValue(
      new Response(JSON.stringify({ detail: "method must be POSHOLD, RTL, or LAND" }), {
        status: 400,
      }),
    );

    await expect(controlTerraSLAMComponent("gps_client", "start")).rejects.toThrow(
      "method must be POSHOLD, RTL, or LAND",
    );
    expect(fetchMock).toHaveBeenCalledWith(
      "/api/control/terraslam/component",
      expect.objectContaining({
        method: "POST",
        body: JSON.stringify({ component: "gps_client", action: "start" }),
      }),
    );
  });

  it("polls calibration progress using the session id", async () => {
    fetchMock.mockResolvedValue(
      new Response(
        JSON.stringify({
          found: true,
          step: "processing_frames",
          frames_done: 4,
          frames_total: 10,
          slam_running: true,
          slam_crashed: false,
        }),
        { status: 200 },
      ),
    );

    const result = await getCalibrationProgress("calib-test");

    expect(result.step).toBe("processing_frames");
    expect(result.frames_done).toBe(4);
    expect(fetchMock).toHaveBeenCalledWith(
      "/api/v1/calibration/session/calib-test/progress",
      expect.anything(),
    );
  });

  it("does not hide a failed calibration process response", async () => {
    fetchMock.mockResolvedValue(
      new Response(JSON.stringify({ detail: "SLAM processing failed" }), { status: 500 }),
    );

    await expect(processCalibrationSession("session-1", "project-1")).rejects.toThrow(
      "SLAM processing failed",
    );
  });

  it("loads a calibration session by id", async () => {
    fetchMock.mockResolvedValue(
      new Response(JSON.stringify({ success: true, session: { id: "session-1", status: "done" } }), {
        status: 200,
      }),
    );

    const result = await getCalibrationSession("session-1");

    expect(result.session.status).toBe("done");
  });
});