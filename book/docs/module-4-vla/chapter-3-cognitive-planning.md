# Chapter 3: Cognitive Planning with LLMs

## Introduction to Cognitive Planning

Cognitive planning represents the "brain" of the Vision-Language-Action (VLA) system, where high-level natural language commands are transformed into executable action sequences that robots can perform. This component bridges the gap between human intentions and robot behaviors, interpreting ambiguous natural language and generating concrete action plans.

The challenge lies in the fact that human language is often ambiguous, context-dependent, and imprecise, while robots require precise, executable commands. Cognitive planning systems must resolve these ambiguities, infer implicit goals, and generate safe, efficient action sequences.

## Role of Large Language Models in Planning

Large Language Models (LLMs) have revolutionized cognitive planning by providing:

- **World knowledge**: LLMs contain vast amounts of information about the world, physics, and common scenarios
- **Natural language understanding**: Ability to parse complex, ambiguous human language
- **Reasoning capabilities**: Logic and inference to determine appropriate actions
- **Adaptability**: Ability to handle novel situations based on learned patterns

LLMs excel at understanding the intent behind human commands and translating them into structured plans, even when the commands are complex or refer to implicit knowledge about the world.

## The Planning Process

### 1. Command Interpretation
The first step involves understanding what the user wants the robot to do:

```python
# Example of command interpretation
user_command = "Please bring me a glass of water from the kitchen"
# System interprets this as:
# - Find a glass object
# - Navigate to kitchen
# - Find water source
# - Fill glass with water
# - Return to user
```

### 2. Context Integration
The system considers:
- Current robot state and capabilities
- Environment map and object positions
- Safety constraints
- Available tools and resources

### 3. Plan Generation
The system generates a sequence of discrete actions:
- Navigation commands
- Object detection tasks
- Manipulation operations
- Verification steps

### 4. Plan Validation
- Verify plan feasibility and safety
- Check for conflicts or impossible states
- Optimize for efficiency and energy

## Implementing LLM-Based Cognitive Planning

Here's a conceptual implementation of cognitive planning using LLMs:

```python
import openai
import json
from typing import List, Dict, Any

class CognitivePlanner:
    def __init__(self, model="gpt-3.5-turbo", api_key=None):
        """
        Initialize the cognitive planner with an LLM
        """
        if api_key:
            openai.api_key = api_key
        self.model = model
        
        # Define robot capabilities
        self.capabilities = [
            "navigate_to",
            "find_object",
            "grasp_object", 
            "release_object",
            "speak",
            "wait",
            "detect_people",
            "open_door",
            "close_door"
        ]
        
    def generate_plan(self, 
                     command: str, 
                     robot_state: Dict[str, Any],
                     environment_map: Dict[str, Any],
                     additional_context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Generate a cognitive plan from a natural language command
        """
        # Prepare the context for the LLM
        context = {
            "capabilities": self.capabilities,
            "robot_state": robot_state,
            "environment_map": environment_map,
            "additional_context": additional_context or {}
        }
        
        # Create the prompt for the LLM
        prompt = self._create_planning_prompt(command, context)
        
        try:
            # Call the LLM
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent planning
                max_tokens=1000
            )
            
            # Parse the response
            plan_text = response.choices[0].message.content
            plan = self._parse_plan(plan_text)
            
            return {
                "success": True,
                "plan": plan,
                "raw_response": plan_text,
                "command": command
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "command": command
            }
    
    def _create_planning_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """
        Create the prompt for the LLM with all necessary context
        """
        prompt = f"""
        Human command: "{command}"

        Robot capabilities: {json.dumps(self.capabilities, indent=2)}
        
        Current robot state: {json.dumps(context["robot_state"], indent=2)}
        
        Environment map: {json.dumps(context["environment_map"], indent=2)}
        
        Additional context: {json.dumps(context["additional_context"], indent=2)}

        Generate a step-by-step action plan using only the robot's capabilities that will satisfy the human command. 
        Each step should be specific, executable, and safe.
        
        Provide your response as a JSON array of action objects with these fields:
        - action: the robot capability to use
        - parameters: any parameters needed for the action (optional)
        - description: human-readable description of the step
        """
        
        return prompt
    
    def _get_system_prompt(self) -> str:
        """
        System prompt to guide the LLM's planning behavior
        """
        return """
        You are an expert robotic planning system. Your role is to convert natural language commands into 
        step-by-step action plans for humanoid robots. Use only the capabilities provided and ensure all 
        plans are safe, feasible, and logically ordered. Consider the robot's current state and environment 
        when generating plans. If a command is ambiguous or impossible, explain why and suggest alternatives 
        when possible.
        """
    
    def _parse_plan(self, plan_text: str) -> List[Dict[str, Any]]:
        """
        Parse the LLM's response into a structured plan
        """
        try:
            # Try to extract JSON from the response
            start = plan_text.find('[')
            end = plan_text.rfind(']') + 1
            
            if start != -1 and end != 0:
                json_str = plan_text[start:end]
                plan = json.loads(json_str)
                
                # Validate plan structure
                for step in plan:
                    if "action" not in step:
                        raise ValueError(f"Plan step missing 'action': {step}")
                
                return plan
            else:
                # If no JSON found, try to parse as plain text
                return self._parse_as_text(plan_text)
        except json.JSONDecodeError:
            return self._parse_as_text(plan_text)
    
    def _parse_as_text(self, plan_text: str) -> List[Dict[str, Any]]:
        """
        Fallback parser for non-JSON responses
        """
        # This is a simplified fallback parser
        # In practice, you'd want more sophisticated text parsing
        lines = plan_text.split('\n')
        plan = []
        
        for line in lines:
            if line.strip().startswith('-') or line.strip().startswith('*'):
                # Simplified parsing - in practice would be more robust
                plan.append({
                    "action": "speak",  # Default to speak for unknown actions
                    "description": line.strip('- *'),
                    "parameters": {}
                })
        
        return plan

# Example usage
if __name__ == "__main__":
    # Initialize the cognitive planner
    planner = CognitivePlanner(api_key="your-openai-api-key")
    
    # Define robot capabilities and state
    robot_state = {
        "location": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "battery_level": 0.85,
        "current_task": None,
        "gripper_status": "open"  # or "closed"
    }
    
    # Define environment map
    environment_map = {
        "rooms": {
            "kitchen": {"x": 5.0, "y": 3.0, "theta": 0.0},
            "living_room": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "bedroom": {"x": -3.0, "y": 2.0, "theta": 0.0}
        },
        "objects": {
            "glass": {"location": {"room": "kitchen", "x": 5.2, "y": 3.1}},
            "water_source": {"location": {"room": "kitchen", "x": 5.5, "y": 3.3}}
        },
        "obstacles": []  # Any static obstacles in the environment
    }
    
    # Generate a plan for a sample command
    command = "Please bring me a glass of water from the kitchen"
    result = planner.generate_plan(command, robot_state, environment_map)
    
    if result["success"]:
        print(f"Command: {result['command']}")
        print("Generated plan:")
        for i, step in enumerate(result["plan"]):
            print(f"  {i+1}. {step['action']}: {step['description']}")
            if 'parameters' in step and step['parameters']:
                print(f"     Parameters: {step['parameters']}")
    else:
        print(f"Planning failed: {result['error']}")
```

## Handling Ambiguity and Uncertainty

### Resolving Ambiguous Commands
- Use context to disambiguate references ("the glass" vs "a glass")
- Ask clarifying questions when necessary
- Make informed assumptions based on common scenarios

### Managing Uncertainty
- Include verification steps in plans
- Plan for potential failures and alternatives
- Use probabilistic reasoning where appropriate

## Safety and Validation in Planning

### Safety Constraints
- Prevent plans that could harm humans or property
- Ensure robot stays within operational limits
- Include collision avoidance in navigation plans

### Plan Validation
- Verify all actions are supported by the robot
- Check that prerequisites for each action are met
- Validate that the final goal state matches the intent

## Integration with Other VLA Components

Cognitive planning must coordinate with other VLA components:

- **With Voice Ingestion**: Refine understanding based on confidence scores
- **With Navigation**: Ensure planned paths are feasible
- **With Perception**: Confirm object locations and states during execution
- **With Manipulation**: Generate appropriate grasping and movement commands

## Challenges and Considerations

### Computational Requirements
- LLM-based planning can be computationally intensive
- Consider edge vs. cloud processing trade-offs
- Optimize for latency requirements of real-time applications

### Context Window Limitations
- LLMs have limited context windows
- Need to summarize or compress complex environments
- Consider multi-step planning approaches

### Reliability
- LLMs can hallucinate or generate incorrect plans
- Implement validation mechanisms
- Use ensemble approaches for critical applications

## Chapter Summary

Cognitive planning with LLMs represents a powerful approach to bridging the gap between natural human language and robot actions. By providing appropriate context and constraints, LLMs can generate detailed action plans that allow humanoid robots to execute complex, natural language commands. Effective implementation requires careful attention to safety, validation, and integration with other VLA components. As LLM technology continues to evolve, we can expect even more sophisticated and reliable cognitive planning capabilities in VLA systems.