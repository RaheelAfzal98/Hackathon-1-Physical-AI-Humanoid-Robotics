# API Contracts for Vision-Language-Action (VLA) Module

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) system that enables voice-to-action capabilities in humanoid robots.

## Voice Processing API

### POST /api/vla/voice/process
Process a voice command and convert it to text using OpenAI Whisper

**Request:**
```
Content-Type: multipart/form-data
Body:
- audio: binary audio file (WAV, MP3, etc.)
- language: optional language code (e.g., "en", "es", "fr")
- model: optional Whisper model type (e.g., "base", "small", "medium", "large")
```

**Response:**
```json
{
  "transcript": "text transcription of the voice command",
  "confidence": 0.95,
  "duration": 2.5,
  "timestamp": "2025-12-11T10:30:00Z"
}
```

**Status Codes:**
- 200: Success
- 400: Invalid request (missing audio file, unsupported format)
- 429: Rate limited by OpenAI
- 500: Internal server error

## Cognitive Planning API

### POST /api/vla/planning/generate
Generate a cognitive plan from natural language command using LLMs

**Request:**
```
Content-Type: application/json
Body:
{
  "command": "natural language command",
  "robot_capabilities": ["navigate", "grasp", "speak"],
  "environment_map": "map identifier or object",
  "context": "additional context about current situation"
}
```

**Response:**
```json
{
  "plan_id": "unique identifier for the plan",
  "steps": [
    {
      "step_id": "unique identifier for the step",
      "action": "action to perform",
      "parameters": {"param1": "value1", "param2": "value2"},
      "description": "human-readable description of the step",
      "estimated_duration": 10,
      "dependencies": ["step_id_1", "step_id_2"]
    }
  ],
  "confidence": 0.85,
  "timestamp": "2025-12-11T10:30:00Z"
}
```

**Status Codes:**
- 200: Success
- 400: Invalid request (missing command, invalid format)
- 500: Internal server error (LLM failure)

## Action Execution API

### POST /api/vla/actions/execute
Execute a sequence of ROS 2 actions

**Request:**
```
Content-Type: application/json
Body:
{
  "plan_id": "identifier of the plan to execute",
  "actions": [
    {
      "action_name": "name of the ROS action",
      "parameters": {"param1": "value1", "param2": "value2"}
    }
  ],
  "execution_options": {
    "priority": "high|medium|low",
    "timeout": 60,
    "allow_fallbacks": true
  }
}
```

**Response:**
```json
{
  "execution_id": "unique identifier for the execution",
  "status": "pending|running|completed|failed",
  "results": [
    {
      "action_name": "name of the ROS action",
      "status": "completed|failed",
      "result_details": "additional details about the result"
    }
  ],
  "timestamp": "2025-12-11T10:30:00Z"
}
```

**Status Codes:**
- 200: Execution started successfully
- 400: Invalid request (missing actions, invalid parameters)
- 500: Internal server error (ROS communication failure)

## Capstone Project API

### POST /api/vla/capstone/execute
Execute the complete Autonomous Humanoid Capstone project

**Request:**
```
Content-Type: application/json
Body:
{
  "voice_command": "natural language command from user",
  "project_config": {
    "environment": "simulation|real_world",
    "safety_constraints": ["max_velocity", "no_collision"],
    "capabilities_to_test": ["navigation", "grasping", "planning"]
  }
}
```

**Response:**
```json
{
  "capstone_id": "unique identifier for the capstone execution",
  "status": "initialized|processing|completed|failed",
  "pipeline": {
    "voice_ingestion": {"status": "completed", "transcript": "..."},
    "cognitive_planning": {"status": "in_progress", "plan_id": "..."},
    "navigation": {"status": "pending"},
    "perception": {"status": "pending"},
    "manipulation": {"status": "pending"}
  },
  "progress": 0.25,
  "timestamp": "2025-12-11T10:30:00Z"
}
```

**Status Codes:**
- 200: Capstone execution started successfully
- 400: Invalid request (missing command, invalid configuration)
- 500: Internal server error