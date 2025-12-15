# Chapter 2: Cognitive Planning

## Introduction

Cognitive planning represents the intelligence layer of Vision-Language-Action (VLA) systems, bridging natural language understanding with robotic action execution. This chapter explores how to use Large Language Models (LLMs) to translate high-level natural language instructions into detailed, executable action plans for robotic systems.

Unlike simple command mapping, cognitive planning involves understanding the context, reasoning about the environment, and generating complex sequences of actions to achieve user goals. This capability is essential for creating robots that can operate in dynamic, unstructured environments where pre-programmed behaviors are insufficient.

## Understanding LLM Integration for Robotics

Large Language Models excel at understanding natural language and generating structured outputs, making them ideal for cognitive planning in robotics. However, integrating LLMs with robotic systems requires careful consideration of safety, reliability, and real-world constraints.

### Prompt Engineering for Action Planning

The key to effective cognitive planning lies in well-designed prompts that guide the LLM to generate appropriate robotic actions. A good prompt structure includes:

1. **System Context**: Information about the robot's capabilities and environment
2. **User Goal**: The natural language instruction to be executed
3. **Action Constraints**: Safety limits and operational boundaries
4. **Output Format**: Structured format for the planned actions

Example prompt template for cognitive planning:

```python
def create_cognitive_planning_prompt(robot_capabilities: dict,
                                   environment_state: dict,
                                   user_instruction: str) -> str:
    """
    Create a prompt for LLM-based cognitive planning
    """
    prompt = f"""
    You are a cognitive planning system for a robotic agent. Your task is to
    translate natural language instructions into executable action sequences.

    ROBOT CAPABILITIES:
    - Mobility: {robot_capabilities.get('mobility', 'Unknown')}
    - Manipulation: {robot_capabilities.get('manipulation', 'Unknown')}
    - Sensors: {robot_capabilities.get('sensors', 'Unknown')}
    - Communication: {robot_capabilities.get('communication', 'Unknown')}

    ENVIRONMENT STATE:
    - Known locations: {environment_state.get('locations', [])}
    - Available objects: {environment_state.get('objects', [])}
    - Current position: {environment_state.get('current_position', 'Unknown')}

    USER INSTRUCTION: {user_instruction}

    Please generate a sequence of actions to fulfill the user's request.
    Each action should be specific and executable by the robot.
    Format your response as a JSON list of actions with the following structure:
    [
        {{
            "action_type": "navigation|manipulation|perception|communication",
            "target": "specific target or location",
            "parameters": {{"key": "value"}},
            "reasoning": "brief explanation of why this action is needed"
        }}
    ]

    Ensure all actions are safe and feasible given the robot's capabilities.
    """
    return prompt
```

### LLM Response Processing

Once the LLM generates a plan, it must be processed and validated before execution:

```python
import json
import re

def process_llm_plan(llm_response: str) -> list:
    """
    Extract and validate action plan from LLM response
    """
    # Extract JSON from response (in case it's embedded in text)
    json_match = re.search(r'\[.*\]', llm_response, re.DOTALL)
    if not json_match:
        raise ValueError("No valid action plan found in LLM response")

    try:
        action_plan = json.loads(json_match.group())
        return validate_action_plan(action_plan)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in action plan: {e}")

def validate_action_plan(action_plan: list) -> list:
    """
    Validate the action plan for safety and feasibility
    """
    validated_plan = []

    for action in action_plan:
        # Validate action structure
        if 'action_type' not in action or 'target' not in action:
            raise ValueError(f"Invalid action structure: {action}")

        # Validate action type
        valid_action_types = ['navigation', 'manipulation', 'perception', 'communication']
        if action['action_type'] not in valid_action_types:
            raise ValueError(f"Invalid action type: {action['action_type']}")

        # Add to validated plan
        validated_plan.append(action)

    return validated_plan
```

## Cognitive Planning Architecture

A robust cognitive planning system requires several components working together:

### 1. Natural Language Understanding Module
Processes the user's natural language instruction to extract key elements such as:

- **Goal**: What the user wants to achieve
- **Objects**: Specific items involved in the task
- **Locations**: Where actions should take place
- **Constraints**: Any limitations or preferences

### 2. World Model Interface
Maintains and updates information about the robot's environment, including:

- Object locations and states
- Navigation maps and safe paths
- Robot current state and capabilities
- Task execution history

### 3. Plan Generation Engine
Uses LLMs to generate action sequences based on the user's goal and the current world state. This component must balance:

- Completeness: Achieving the user's goal
- Safety: Avoiding dangerous situations
- Efficiency: Minimizing execution time and energy
- Robustness: Handling unexpected situations

### 4. Plan Validation Module
Validates generated plans against safety constraints and feasibility requirements before execution.

## Integration with ROS 2

Integrating cognitive planning with ROS 2 involves several architectural patterns:

### Action Plan Execution Node

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from .utils import process_llm_plan, create_cognitive_planning_prompt

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner')

        # Publishers and subscribers
        self.instruction_sub = self.create_subscription(
            String,
            'natural_language_instruction',
            self.instruction_callback,
            10
        )

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, MoveItGoal, 'move_group')

        # World model interface
        self.world_model = WorldModelInterface(self)

    def instruction_callback(self, msg: String):
        """
        Process natural language instruction and generate action plan
        """
        try:
            # Get current world state
            world_state = self.world_model.get_current_state()

            # Get robot capabilities
            robot_caps = self.world_model.get_robot_capabilities()

            # Create prompt for LLM
            prompt = create_cognitive_planning_prompt(
                robot_caps, world_state, msg.data
            )

            # Get plan from LLM (this would call your LLM API)
            llm_response = self.call_llm_api(prompt)

            # Process and validate the plan
            action_plan = process_llm_plan(llm_response)

            # Execute the plan
            self.execute_action_plan(action_plan)

        except Exception as e:
            self.get_logger().error(f"Error processing instruction: {e}")

    def execute_action_plan(self, action_plan: list):
        """
        Execute the sequence of actions
        """
        for action in action_plan:
            if action['action_type'] == 'navigation':
                self.execute_navigation(action['target'])
            elif action['action_type'] == 'manipulation':
                self.execute_manipulation(action['target'], action.get('parameters', {}))
            # Add other action types as needed
```

### World Model Interface

The world model provides the cognitive planner with information about the environment:

```python
class WorldModelInterface:
    def __init__(self, node: Node):
        self.node = node
        self.locations = {}
        self.objects = {}
        self.robot_state = {}

        # Subscribe to relevant topics
        self.location_sub = node.create_subscription(
            String, 'known_locations', self.location_callback, 10
        )
        self.object_sub = node.create_subscription(
            String, 'detected_objects', self.object_callback, 10
        )

    def get_current_state(self) -> dict:
        """
        Get current state of the world
        """
        return {
            'locations': list(self.locations.keys()),
            'objects': list(self.objects.keys()),
            'current_position': self.robot_state.get('position', 'unknown'),
            'robot_orientation': self.robot_state.get('orientation', 'unknown')
        }

    def get_robot_capabilities(self) -> dict:
        """
        Get robot capabilities
        """
        return {
            'mobility': 'differential drive',
            'manipulation': '7-DOF arm with gripper',
            'sensors': ['RGB-D camera', 'LiDAR', 'IMU'],
            'communication': 'text-to-speech, LED indicators'
        }
```

## Natural Language Understanding for Robotics

Effective cognitive planning requires sophisticated natural language understanding that goes beyond simple keyword matching:

### Intent Recognition

Understanding what the user wants to achieve:

```python
def recognize_intent(instruction: str) -> dict:
    """
    Recognize the intent and extract relevant information from the instruction
    """
    # Define intent patterns
    intent_patterns = {
        'fetch_object': [
            r'bring me (?:the )?(.+)',
            r'get (?:the )?(.+)',
            r'pick up (?:the )?(.+)',
            r'fetch (?:the )?(.+)'
        ],
        'navigate_to': [
            r'go to (?:the )?(.+)',
            r'move to (?:the )?(.+)',
            r'take me to (?:the )?(.+)',
            r'go (?:to the )?(.+)'
        ],
        'transport_object': [
            r'take (?:the )?(.+) to (?:the )?(.+)',
            r'move (?:the )?(.+) from (.+) to (.+)',
            r'bring (?:the )?(.+) to (?:the )?(.+)'
        ]
    }

    for intent, patterns in intent_patterns.items():
        for pattern in patterns:
            import re
            match = re.search(pattern, instruction, re.IGNORECASE)
            if match:
                return {
                    'intent': intent,
                    'entities': match.groups(),
                    'original_instruction': instruction
                }

    # If no specific pattern matches, return as general instruction
    return {
        'intent': 'general',
        'entities': [instruction],
        'original_instruction': instruction
    }
```

### Contextual Understanding

Understanding how context affects the interpretation of commands:

```python
def apply_contextual_understanding(intent_result: dict,
                                world_state: dict) -> dict:
    """
    Apply contextual understanding to refine the interpretation
    """
    intent = intent_result['intent']
    entities = intent_result['entities']

    if intent == 'fetch_object' and len(entities) == 1:
        object_name = entities[0]

        # Check if object is known in the environment
        if object_name not in world_state['objects']:
            # Try to resolve using LLM
            resolved_object = resolve_ambiguous_object(object_name, world_state)
            if resolved_object:
                entities = [resolved_object]

    elif intent == 'navigate_to' and len(entities) == 1:
        location_name = entities[0]

        # Check if location is known
        if location_name not in world_state['locations']:
            # Try to find closest match
            closest_match = find_closest_location(location_name, world_state)
            if closest_match:
                entities = [closest_match]

    return {
        'intent': intent,
        'entities': entities,
        'original_instruction': intent_result['original_instruction']
    }
```

## Implementation Examples

### Multi-step Task Planning

For complex tasks that require multiple steps:

```python
def plan_multi_step_task(user_instruction: str,
                        world_state: dict,
                        robot_caps: dict) -> list:
    """
    Plan a multi-step task using LLM-based cognitive planning
    """
    prompt = f"""
    User wants: {user_instruction}

    Robot capabilities: {robot_caps}
    Current world state: {world_state}

    Please break this down into a sequence of specific, executable actions.
    Each action should be something the robot can do directly.
    Consider the preconditions and effects of each action.

    Example of a multi-step task:
    Instruction: "Please bring the red cup from the kitchen to the living room"

    Actions:
    1. Navigate to kitchen
    2. Perceive/locate red cup
    3. Grasp red cup
    4. Navigate to living room
    5. Place red cup at appropriate location

    Provide your plan as a JSON list of actions.
    """

    # Call LLM API with the prompt
    llm_response = call_llm_api(prompt)

    # Process and return the action plan
    return process_llm_plan(llm_response)
```

### Safety and Validation Layer

Critical for ensuring safe operation:

```python
def validate_plan_safety(action_plan: list,
                        world_state: dict,
                        robot_caps: dict) -> tuple[bool, list]:
    """
    Validate the action plan for safety
    """
    issues = []

    for i, action in enumerate(action_plan):
        # Check navigation safety
        if action['action_type'] == 'navigation':
            path_safe, reason = check_navigation_safety(action['target'], world_state)
            if not path_safe:
                issues.append(f"Action {i}: Navigation to {action['target']} unsafe - {reason}")

        # Check manipulation safety
        elif action['action_type'] == 'manipulation':
            obj_safe, reason = check_object_safety(action['target'], world_state)
            if not obj_safe:
                issues.append(f"Action {i}: Manipulation of {action['target']} unsafe - {reason}")

    return len(issues) == 0, issues
```

## Best Practices

### 1. Plan Validation
Always validate generated plans against safety constraints and feasibility requirements before execution.

### 2. Context Maintenance
Maintain and update context about the environment and task state to improve planning accuracy.

### 3. Human-in-the-Loop
For critical tasks, implement confirmation mechanisms to involve human oversight.

### 4. Error Recovery
Design plans with built-in error recovery strategies to handle unexpected situations.

### 5. Explainability
Provide explanations for the planned actions to increase user trust and understanding.

## Summary

Cognitive planning enables VLA systems to understand complex natural language instructions and translate them into executable robotic actions. Key components include:

1. LLM-based plan generation with appropriate prompt engineering
2. Integration with ROS 2 for action execution
3. World model for maintaining environmental context
4. Safety validation to ensure reliable operation

The next chapter will focus on integrating voice-to-action and cognitive planning components into a complete VLA system for an autonomous humanoid robot.