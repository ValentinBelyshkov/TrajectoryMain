import { RequestHandler } from "express";
import {
  Project,
  CreateProjectRequest,
  CreateProjectResponse,
  ProjectsResponse,
  ProjectType,
} from "@shared/api";

const projects: Project[] = [];

export const handleGetProjects: RequestHandler = (_req, res) => {
  const response: ProjectsResponse = {
    projects: projects.sort(
      (a, b) =>
        new Date(b.createdAt).getTime() - new Date(a.createdAt).getTime(),
    ),
  };
  res.json(response);
};

export const handleCreateProject: RequestHandler = (req, res) => {
  const { name, type } = req.body as CreateProjectRequest;

  if (!name || !type) {
    const response: CreateProjectResponse = {
      success: false,
      error: "Название и тип проекта обязательны",
    };
    return res.status(400).json(response);
  }

  if (type !== "камера" && type !== "симуляция") {
    const response: CreateProjectResponse = {
      success: false,
      error: "Неверный тип проекта",
    };
    return res.status(400).json(response);
  }

  const newProject: Project = {
    id: Date.now().toString(),
    name,
    type: type as ProjectType,
    createdAt: new Date().toISOString(),
  };

  projects.push(newProject);

  const response: CreateProjectResponse = {
    success: true,
    project: newProject,
  };

  res.json(response);
};

export const handleDeleteProject: RequestHandler = (req, res) => {
  const { id } = req.params;

  const index = projects.findIndex((p) => p.id === id);

  if (index === -1) {
    return res.status(404).json({ success: false, error: "Проект не найден" });
  }

  projects.splice(index, 1);

  res.json({ success: true });
};
