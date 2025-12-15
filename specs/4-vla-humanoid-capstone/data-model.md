# Data Model for Module 4: Vision-Language-Action (VLA)

## Key Entities

### VLASystem
- **ID**: Unique identifier for the VLA system
- **Name**: Name of the VLA system
- **Description**: Description of what the VLA system does
- **CreatedDate**: Date when the VLA system was created
- **LastModified**: Date when the VLA system was last modified
- **Status**: Current status ("Draft", "Published", "Archived")

### VoiceCommand
- **ID**: Unique identifier for the voice command
- **VLASystemID**: Reference to the VLASystem entity
- **AudioData**: Raw audio data from the user
- **Transcript**: Text transcription of the voice command
- **Confidence**: Confidence score of the transcription (0.0-1.0)
- **Processed**: Boolean indicating if the command has been processed
- **ProcessedDate**: Date when the command was processed

### CognitivePlan
- **ID**: Unique identifier for the cognitive plan
- **VLASystemID**: Reference to the VLASystem entity
- **InputCommand**: The original natural language command
- **PlanSteps**: List of steps in the plan
- **GeneratedBy**: Identifier of the LLM that generated the plan
- **CreatedDate**: Date when the plan was generated
- **Status**: Current status ("Generated", "Executing", "Completed", "Failed")

### ROS2ActionSequence
- **ID**: Unique identifier for the ROS 2 action sequence
- **CognitivePlanID**: Reference to the CognitivePlan entity
- **ActionName**: Name of the ROS 2 action
- **Parameters**: Parameters for the action
- **Priority**: Priority of the action in the sequence
- **Dependencies**: List of other actions this action depends on
- **Status**: Current status ("Pending", "Executing", "Completed", "Failed")

### AutonomousHumanoidCapstone
- **ID**: Unique identifier for the capstone project
- **Name**: Name of the capstone project
- **Description**: Description of the capstone project
- **VoiceCommandID**: Reference to associated voice command
- **CognitivePlanID**: Reference to associated cognitive plan
- **CreatedDate**: Date when the capstone was created
- **Status**: Current status ("Draft", "In Progress", "Completed", "Archived")

## Relationships

- VLASystem → VoiceCommand (One-to-Many): A VLA system can receive multiple voice commands
- VLASystem → CognitivePlan (One-to-Many): A VLA system can generate multiple cognitive plans
- CognitivePlan → ROS2ActionSequence (One-to-Many): A cognitive plan can contain multiple ROS 2 action sequences
- VoiceCommand → AutonomousHumanoidCapstone (One-to-Many): A voice command can initiate multiple capstone projects
- CognitivePlan → AutonomousHumanoidCapstone (One-to-Many): A cognitive plan can be part of multiple capstone projects

## Validation Rules

1. Every VLASystem must have a unique name
2. Every VoiceCommand must have a non-empty transcript and confidence score between 0.0 and 1.0
3. Every CognitivePlan must have at least one plan step and be associated with a valid voice command
4. Every ROS2ActionSequence must have a valid action name and priority
5. Every AutonomousHumanoidCapstone must have both a voice command and cognitive plan associated
6. Confidence scores in VoiceCommand must be between 0.0 and 1.0
7. Dependencies in ROS2ActionSequence must reference valid other actions in the same plan
8. Status values must be one of the allowed values for each entity

## State Transitions

- Draft → In Progress: When the capstone project is actively being worked on
- In Progress → Completed: When all tasks in the capstone project are finished
- In Progress → Archived: When the capstone project is abandoned or suspended
- Completed → Archived: When a completed project is no longer active

## Examples

```yaml
VLASystem:
  ID: "vlas001"
  Name: "Autonomous Humanoid Assistant"
  Description: "VLA system for interacting with humans through voice commands"
  CreatedDate: "2025-12-11"
  LastModified: "2025-12-11"
  Status: "Published"

VoiceCommand:
  ID: "vc001"
  VLASystemID: "vlas001"
  AudioData: "audio_file_path"
  Transcript: "Please navigate to the kitchen and bring me a glass of water"
  Confidence: 0.95
  Processed: true
  ProcessedDate: "2025-12-11"

CognitivePlan:
  ID: "cp001"
  VLASystemID: "vlas001"
  InputCommand: "Please navigate to the kitchen and bring me a glass of water"
  PlanSteps:
    - "Parse command to identify objects and destination"
    - "Plan navigation route to kitchen"
    - "Navigate to kitchen"
    - "Identify glass object"
    - "Grasp glass object"
    - "Navigate back to user"
    - "Deliver glass to user"
  GeneratedBy: "GPT-4"
  CreatedDate: "2025-12-11"
  Status: "Generated"

ROS2ActionSequence:
  ID: "ras001"
  CognitivePlanID: "cp001"
  ActionName: "NavigateTo"
  Parameters:
    x: 5.0
    y: 3.0
    theta: 0.0
  Priority: 1
  Dependencies: []
  Status: "Pending"

AutonomousHumanoidCapstone:
  ID: "ahc001"
  Name: "Kitchen Assistant"
  Description: "Capstone project to demonstrate VLA capabilities"
  VoiceCommandID: "vc001"
  CognitivePlanID: "cp001"
  CreatedDate: "2025-12-11"
  Status: "In Progress"
```