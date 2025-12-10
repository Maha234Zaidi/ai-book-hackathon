# Detailed Documentation: Cognitive Planning Implementation

## Overview

Cognitive planning represents a critical component of Vision-Language-Action (VLA) systems, bridging the gap between high-level natural language commands and executable robotic actions. This document provides comprehensive documentation for implementing cognitive planning with Large Language Models (LLMs), covering design principles, implementation details, and best practices.

## System Architecture

### High-Level Architecture

The cognitive planning system consists of several interconnected components:

```
Natural Language Input → Preprocessing → LLM Processing → Action Translation → Validation → Robot Execution
         ↑                    ↓               ↓              ↓              ↓            ↓
Environment Context ← Perception Data ← Context Integration ← Safety Check ← Execution ← Action Feedback
```

### Component Breakdown

#### 1. Natural Language Interface
- **Purpose**: Receives and processes natural language commands
- **Responsibilities**:
  - Command parsing and normalization
  - Intent identification
  - Parameter extraction
  - Ambiguity resolution
- **Interface**: ROS 2 topic `/vla/command` or service

#### 2. Context Integrator
- **Purpose**: Incorporates environmental and situational context
- **Responsibilities**:
  - Aggregates perception data
  - Maintains world state
  - Updates with real-time environmental changes
- **Interface**: Multiple ROS 2 topics for different context sources

#### 3. Planning Engine
- **Purpose**: Transforms commands into action sequences using LLM
- **Responsibilities**:
  - LLM-based task decomposition
  - Action sequence generation
  - Reasoning and decision making
  - Plan optimization
- **Interface**: OpenAI API or other LLM provider

#### 4. Action Validator
- **Purpose**: Ensures action sequences are safe and feasible
- **Responsibilities**:
  - Safety validation
  - Feasibility checks
  - Environmental constraint verification
  - Risk assessment
- **Interface**: Internal validation API

#### 5. Action Sequencer
- **Purpose**: Orchestrates execution of validated action sequences
- **Responsibilities**:
  - Execution scheduling
  - Progress monitoring
  - Error handling and recovery
  - Status reporting
- **Interface**: ROS 2 action servers and topics

## Detailed Implementation

### Core Planning Engine

The core of the cognitive planning system is implemented in the PlanningEngine class:

```python
class PlanningEngine:
    """
    Main cognitive planning engine that processes natural language into action sequences.
    """
    
    def __init__(self, llm_config: dict):
        self.llm_client = LLMClient(config=llm_config)
        self.context_manager = ContextManager()
        self.action_validator = ActionValidator()
        self.logger = logging.getLogger(__name__)
        
        # Initialize LLM and planning parameters
        self.model = llm_config.get('model', 'gpt-3.5-turbo')
        self.temperature = llm_config.get('temperature', 0.3)
        self.max_tokens = llm_config.get('max_tokens', 1000)
        self.timeout = llm_config.get('timeout', 30)
        
        # Set up supported action types
        self.action_catalog = ActionCatalog()
    
    def plan_task(self, 
                  command: str, 
                  context: Dict[str, Any], 
                  options: PlanningOptions = None) -> PlanningResult:
        """
        Plan a task by converting natural language to action sequence.
        
        Args:
            command: Natural language command to plan
            context: Environmental and situational context
            options: Additional planning options
            
        Returns:
            PlanningResult with action sequence and metadata
        """
        try:
            # 1. Preprocess command
            processed_command = self._preprocess_command(command)
            
            # 2. Integrate context for planning
            enriched_context = self.context_manager.enrich_context(context)
            
            # 3. Generate plan using LLM
            raw_plan = self._generate_plan_with_llm(processed_command, enriched_context, options)
            
            # 4. Parse LLM response into action sequence
            action_sequence = self._parse_llm_response(raw_plan)
            
            # 5. Validate the generated sequence
            validation_result = self.action_validator.validate(action_sequence, enriched_context)
            
            # 6. Generate final planning result
            result = PlanningResult(
                action_sequence=action_sequence,
                reasoning=raw_plan.get('reasoning', ''),
                confidence=raw_plan.get('confidence', 0.0),
                validation_result=validation_result,
                success=True,
                metadata={
                    'model_used': self.model,
                    'processing_time': time.time() - start_time
                }
            )
            
            self.logger.info(f"Plan generated with {len(action_sequence)} steps")
            return result
            
        except Exception as e:
            self.logger.error(f"Error in planning task: {str(e)}")
            return PlanningResult(success=False, error=str(e))
    
    def _preprocess_command(self, command: str) -> str:
        """
        Preprocess the command for better LLM understanding.
        """
        # Normalize command
        command = command.lower().strip()
        
        # Expand abbreviations
        expansions = {
            'wanna': 'want to',
            'gonna': 'going to',
            'gimme': 'give me',
            'lemme': 'let me',
            'kinda': 'kind of'
        }
        
        for abbrev, full in expansions.items():
            command = re.sub(r'\b' + abbrev + r'\b', full, command)
        
        return command
    
    def _generate_plan_with_llm(self, 
                               command: str, 
                               context: Dict[str, Any], 
                               options: PlanningOptions) -> Dict[str, Any]:
        """
        Generate plan using Large Language Model.
        """
        # Create structured prompt
        prompt = self._create_planning_prompt(command, context, options)
        
        # Call LLM
        response = self.llm_client.chat_completion(
            model=self.model,
            messages=[
                {"role": "system", "content": self._get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            timeout=self.timeout
        )
        
        # Extract and validate response
        content = response.choices[0].message.content
        parsed_response = self._parse_llm_response(content)
        
        return parsed_response
    
    def _create_planning_prompt(self, 
                               command: str, 
                               context: Dict[str, Any], 
                               options: PlanningOptions) -> str:
        """
        Create structured prompt for the LLM.
        """
        context_str = json.dumps(context, indent=2)
        
        # Determine planning style based on options
        style_instruction = "Create a detailed, step-by-step plan." if options.detailed else "Create a concise plan."
        
        # Include safety and constraint information
        constraints_str = self._format_constraints(options)
        
        prompt = f"""
        You are an expert cognitive planner for a robotic system. Convert the following natural language command into a structured action sequence.
        
        Environment Context:
        {context_str}
        
        Command: "{command}"
        
        Constraints and Requirements:
        {constraints_str}
        
        Instructions: {style_instruction}
        
        Provide your response as a JSON object with these keys:
        - "action_sequence": Array of action objects with type and parameters
        - "reasoning": Explanation of your plan
        - "confidence": Your confidence in the plan (0.0-1.0)
        - "safety_considerations": Potential safety issues and mitigations
        
        Supported action types: {', '.join(self.action_catalog.get_supported_types())}
        
        Example format:
        {{
            "action_sequence": [
                {{"action_type": "navigate_to_location", "parameters": {{"x": 1.0, "y": 2.0, "frame": "map"}}}},
                {{"action_type": "detect_object", "parameters": {{"object_type": "cup"}}}},
                {{"action_type": "grasp_object", "parameters": {{"object_name": "cup", "grasp_type": "top"}}}}
            ],
            "reasoning": "First navigate to the known location of the cup, then detect it to confirm its presence and exact position, then grasp it using a top grasp approach.",
            "confidence": 0.85,
            "safety_considerations": ["Check for obstacles during navigation", "Verify cup is not hot before grasping"]
        }}
        """
        
        return prompt
    
    def _parse_llm_response(self, response: str) -> Dict[str, Any]:
        """
        Parse the LLM response into structured plan.
        """
        try:
            # Extract JSON from response if it contains markdown
            json_match = re.search(r'```json\s*(.*?)\s*```', response, re.DOTALL)
            if json_match:
                json_str = json_match.group(1)
            else:
                # Find JSON object in the response
                brace_count = 0
                start_idx = response.find('{')
                if start_idx != -1:
                    for i, char in enumerate(response[start_idx:], start_idx):
                        if char == '{':
                            brace_count += 1
                        elif char == '}':
                            brace_count -= 1
                            if brace_count == 0:
                                json_str = response[start_idx:i+1]
                                break
                    else:
                        json_str = response[start_idx:]  # Assume everything after { is JSON
                else:
                    raise ValueError("No JSON object found in response")
            
            parsed = json.loads(json_str)
            
            # Validate required fields
            required = ['action_sequence', 'reasoning', 'confidence']
            for field in required:
                if field not in parsed:
                    self.logger.warning(f"Field '{field}' missing from LLM response")
            
            return parsed
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse LLM response as JSON: {e}")
            self.logger.debug(f"Response content: {response}")
            return {}
        except Exception as e:
            self.logger.error(f"Error parsing LLM response: {str(e)}")
            return {}

class PlanningOptions:
    """
    Options for cognitive planning.
    """
    def __init__(self,
                 detailed: bool = False,
                 safety_required: bool = True,
                 constraints: List[str] = None,
                 time_limit: float = None,
                 priority: int = 1):
        self.detailed = detailed
        self.safety_required = safety_required
        self.constraints = constraints or []
        self.time_limit = time_limit
        self.priority = priority

class PlanningResult:
    """
    Result of cognitive planning.
    """
    def __init__(self,
                 action_sequence: List[ActionStep] = None,
                 reasoning: str = "",
                 confidence: float = 0.0,
                 validation_result: ValidationResult = None,
                 success: bool = True,
                 error: str = None,
                 metadata: Dict[str, Any] = None):
        self.action_sequence = action_sequence or []
        self.reasoning = reasoning
        self.confidence = confidence
        self.validation_result = validation_result
        self.success = success
        self.error = error
        self.metadata = metadata or {}
```

### Context Management System

The context management system maintains and updates environmental information:

```python
class ContextManager:
    """
    Manages environmental, perceptual, and situational context for planning.
    """
    
    def __init__(self):
        self.knowledge_base = KnowledgeBase()
        self.world_state = WorldState()
        self.perception_buffer = PerceptionBuffer()
        self.logger = logging.getLogger(__name__)
    
    def enrich_context(self, base_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Enrich a base context with additional information from various sources.
        
        Args:
            base_context: Initial environmental context
            
        Returns:
            Enriched context with additional information
        """
        enriched = base_context.copy()
        
        # Add world state information
        world_state_info = self.world_state.get_current_state()
        enriched.update(world_state_info)
        
        # Add perception data
        perception_data = self.perception_buffer.get_latest_perceptions()
        enriched.setdefault('perception', []).extend(perception_data)
        
        # Add temporal context
        enriched['timestamp'] = time.time()
        enriched['time_of_day'] = self._get_time_of_day()
        
        # Add robot state
        enriched['robot_state'] = self._get_robot_state()
        
        # Add task history (for multi-step planning)
        recent_tasks = self.knowledge_base.get_recent_tasks()
        enriched['task_history'] = recent_tasks
        
        # Add domain knowledge
        domain_knowledge = self.knowledge_base.get_domain_knowledge(enriched.get('domain', 'general'))
        enriched['domain_knowledge'] = domain_knowledge
        
        return enriched
    
    def update_perception(self, perception_data: Dict[str, Any]):
        """
        Update the context with new perception information.
        """
        self.perception_buffer.add_perception(perception_data)
        
        # Update world state based on perception
        self.world_state.update_from_perception(perception_data)
        
        # Trigger context change notifications
        self._notify_context_change('perception', perception_data)
    
    def _get_robot_state(self) -> Dict[str, Any]:
        """
        Get current robot state information.
        """
        # This would normally interface with robot's state estimator
        return {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'theta': 0.0},
            'battery_level': 85,
            'gripper_status': 'open',
            'current_task': None,
            'action_history': []
        }
    
    def _get_time_of_day(self) -> str:
        """
        Get current time of day for context.
        """
        hour = datetime.now().hour
        if 5 <= hour < 12:
            return 'morning'
        elif 12 <= hour < 17:
            return 'afternoon'
        elif 17 <= hour < 21:
            return 'evening'
        else:
            return 'night'

class PerceptionBuffer:
    """
    Buffer for storing recent perception data.
    """
    
    def __init__(self, max_items: int = 100):
        self.buffer = deque(maxlen=max_items)
        self.types_index = {}  # Index by perception type
    
    def add_perception(self, perception: Dict[str, Any]):
        """
        Add a perception to the buffer.
        """
        perception['timestamp'] = time.time()
        self.buffer.append(perception)
        
        # Update index by type
        ptype = perception.get('type', 'unknown')
        if ptype not in self.types_index:
            self.types_index[ptype] = deque(maxlen=10)  # Keep only recent of each type
        self.types_index[ptype].append(perception)
    
    def get_latest_perceptions(self, max_age: float = 1.0) -> List[Dict[str, Any]]:
        """
        Get recent perceptions within age threshold.
        """
        current_time = time.time()
        recent = [
            p for p in self.buffer 
            if current_time - p.get('timestamp', 0) <= max_age
        ]
        return recent

class WorldState:
    """
    Maintains current state of the world for planning.
    """
    
    def __init__(self):
        self.objects = {}  # {id: object_info}
        self.locations = {}  # {name: location_info}
        self.relationships = set()  # Set of spatial/functional relationships
        self.dynamic_properties = {}  # Properties that change frequently
    
    def update_from_perception(self, perception: Dict[str, Any]):
        """
        Update world state based on a perception.
        """
        ptype = perception.get('type')
        if ptype == 'object_detection':
            self._update_object_state(perception)
        elif ptype == 'location_identification':
            self._update_location_state(perception)
        elif ptype == 'spatial_relationship':
            self._update_relationship_state(perception)
    
    def _update_object_state(self, detection: Dict[str, Any]):
        """
        Update object information in world state.
        """
        obj_id = detection.get('object_id')
        if obj_id:
            # Merge with existing object info or create new
            if obj_id in self.objects:
                self.objects[obj_id].update(detection)
                self.objects[obj_id]['last_seen'] = time.time()
            else:
                self.objects[obj_id] = {
                    'properties': detection.get('properties', {}),
                    'location': detection.get('position', {}),
                    'last_seen': time.time(),
                    'last_updated': time.time()
                }
    
    def get_current_state(self) -> Dict[str, Any]:
        """
        Get a snapshot of the current world state.
        """
        return {
            'objects': dict(self.objects),
            'locations': dict(self.locations),
            'relationships': list(self.relationships),
            'dynamic_properties': dict(self.dynamic_properties)
        }
```

### Action Validation System

The action validation system ensures plans are safe and executable:

```python
class ActionValidator:
    """
    Validates action sequences for safety and feasibility.
    """
    
    def __init__(self):
        self.safety_checker = SafetyChecker()
        self.executor = ActionExecutor()
        self.logger = logging.getLogger(__name__)
    
    def validate(self, 
                action_sequence: List[ActionStep], 
                environment_context: Dict[str, Any]) -> ValidationResult:
        """
        Validate an action sequence for safety and feasibility.
        
        Args:
            action_sequence: Sequence of actions to validate
            environment_context: Current environmental context
            
        Returns:
            ValidationResult with validation status and issues
        """
        start_time = time.time()
        issues = []
        warnings = []
        
        # Check overall sequence validity
        if not action_sequence:
            issues.append("Action sequence is empty")
            return ValidationResult(
                is_valid=False,
                issues=issues,
                warnings=warnings,
                validation_time=time.time() - start_time
            )
        
        # Validate each action in the sequence
        previous_state = self._initialize_state(environment_context)
        for i, action in enumerate(action_sequence):
            # Validate the action itself
            action_issues, action_warnings = self._validate_single_action(
                action, environment_context, previous_state
            )
            
            # Prepend step number to issue/warning messages
            action_issues = [f"[Step {i}]: {issue}" for issue in action_issues]
            action_warnings = [f"[Step {i}]: {warning}" for warning in action_warnings]
            
            issues.extend(action_issues)
            warnings.extend(action_warnings)
            
            # Update state based on action effects
            previous_state = self._update_state(previous_state, action)
        
        # Check sequence-level constraints
        sequence_issues = self._validate_sequence_constraints(action_sequence, environment_context)
        issues.extend(sequence_issues)
        
        # Assess overall safety
        safety_level, risk_level = self._assess_safety_level(issues, warnings)
        
        # Calculate confidence in validation
        confidence = self._calculate_validation_confidence(issues, warnings)
        
        validation_time = time.time() - start_time
        
        return ValidationResult(
            is_valid=(safety_level != SafetyLevel.DANGEROUS and risk_level != RiskLevel.HIGH),
            safety_level=safety_level,
            risk_level=risk_level,
            issues=issues,
            warnings=warnings,
            confidence=confidence,
            validation_time=validation_time
        )
    
    def _validate_single_action(self, 
                               action: ActionStep, 
                               context: Dict[str, Any], 
                               current_state: Dict[str, Any]) -> Tuple[List[str], List[str]]:
        """
        Validate a single action step.
        
        Args:
            action: Action to validate
            context: Environmental context
            current_state: Current world state
            
        Returns:
            Tuple of (issues, warnings)
        """
        issues = []
        warnings = []
        
        # Check action type validity
        if not self.executor.is_valid_action_type(action.action_type):
            issues.append(f"Invalid action type: {action.action_type}")
            return issues, warnings
        
        # Perform action-specific validation
        validator_func = getattr(self, f'_validate_{action.action_type}', self._validate_generic)
        action_issues, action_warnings = validator_func(action, context, current_state)
        
        issues.extend(action_issues)
        warnings.extend(action_warnings)
        
        # Check safety constraints
        safety_issues = self.safety_checker.check_action_safety(action, context)
        issues.extend([f"Safety: {issue}" for issue in safety_issues])
        
        return issues, warnings
    
    def _validate_navigate_to_location(self, 
                                     action: ActionStep, 
                                     context: Dict[str, Any], 
                                     current_state: Dict[str, Any]) -> Tuple[List[str], List[str]]:
        """
        Validate navigation action.
        """
        issues = []
        warnings = []
        
        # Check required parameters
        if 'x' not in action.parameters or 'y' not in action.parameters:
            issues.append("Missing required coordinates (x, y)")
            return issues, warnings
        
        x, y = action.parameters['x'], action.parameters['y']
        
        # Validate coordinate ranges
        if not (-100 <= x <= 100) or not (-100 <= y <= 100):
            issues.append(f"Coordinates outside safe range: ({x}, {y})")
        
        # Check for obstacles in path
        obstacles = context.get('obstacles', [])
        path_clear = self._check_path_obstacles(x, y, obstacles)
        if not path_clear:
            warnings.append(f"Path to ({x}, {y}) may have obstacles")
        
        # Check if destination is accessible
        navigable_areas = context.get('navigable_areas', [])
        destination_accessible = self._is_destination_accessible(x, y, navigable_areas)
        if not destination_accessible:
            issues.append(f"Destination ({x}, {y}) is not in accessible area")
        
        return issues, warnings
    
    def _validate_grasp_object(self, 
                             action: ActionStep, 
                             context: Dict[str, Any], 
                             current_state: Dict[str, Any]) -> Tuple[List[str], List[str]]:
        """
        Validate object grasping action.
        """
        issues = []
        warnings = []
        
        # Check required parameters
        if 'object_name' not in action.parameters:
            issues.append("Missing required object_name parameter")
        
        obj_name = action.parameters.get('object_name')
        if obj_name:
            # Check if object exists and is graspable
            obj_info = self._find_object_by_name(obj_name, context.get('objects', []))
            if not obj_info:
                issues.append(f"Object '{obj_name}' not found in environment")
            elif not obj_info.get('graspable', True):
                issues.append(f"Object '{obj_name}' is not graspable")
            
            # Check object properties
            if obj_info and obj_info.get('weight', 0) > MAX_GRASP_WEIGHT:
                issues.append(f"Object '{obj_name}' is too heavy to grasp")
            
            if obj_info and obj_info.get('fragility') == 'fragile':
                warnings.append(f"Object '{obj_name}' is fragile, using gentle grasp")
        
        # Check grasp type
        grasp_type = action.parameters.get('grasp_type', 'default')
        valid_grasps = ['top', 'side', 'edge', 'pinch', 'default']
        if grasp_type not in valid_grasps:
            warnings.append(f"Unusual grasp type: {grasp_type}")
        
        return issues, warnings
    
    def _validate_generic(self, 
                         action: ActionStep, 
                         context: Dict[str, Any], 
                         current_state: Dict[str, Any]) -> Tuple[List[str], List[str]]:
        """
        Generic action validation for unsupported action types.
        """
        return [], [f"Generic validation performed for unsupported action type: {action.action_type}"]
    
    def _validate_sequence_constraints(self, 
                                     action_sequence: List[ActionStep], 
                                     context: Dict[str, Any]) -> List[str]:
        """
        Validate constraints at the sequence level.
        """
        issues = []
        
        # Check for dangerous action patterns
        for i in range(len(action_sequence) - 1):
            current = action_sequence[i]
            next_action = action_sequence[i + 1]
            
            # Check for unsafe transitions
            if (current.action_type == 'grasp_object' and 
                next_action.action_type == 'move_base' and
                next_action.parameters.get('speed') == 'fast'):
                issues.append(
                    f"Unsafe transition at step {i}->{i+1}: "
                    f"fast movement after grasping object"
                )
        
        # Check sequence length
        max_length = context.get('max_sequence_length', 50)
        if len(action_sequence) > max_length:
            issues.append(
                f"Action sequence too long: {len(action_sequence)} > {max_length} max steps"
            )
        
        # Check total estimated execution time
        total_time = sum(action.estimated_duration for action in action_sequence)
        max_time = context.get('max_execution_time', 300)  # 5 minutes default
        if total_time > max_time:
            issues.append(
                f"Estimated execution time too long: {total_time}s > {max_time}s max"
            )
        
        return issues

class ValidationResult:
    """
    Result of action sequence validation.
    """
    def __init__(self,
                 is_valid: bool = True,
                 safety_level: SafetyLevel = SafetyLevel.UNKNOWN,
                 risk_level: RiskLevel = RiskLevel.ACCEPTABLE,
                 issues: List[str] = None,
                 warnings: List[str] = None,
                 confidence: float = 1.0,
                 validation_time: float = 0.0):
        self.is_valid = is_valid
        self.safety_level = safety_level or SafetyLevel.UNKNOWN
        self.risk_level = risk_level or RiskLevel.ACCEPTABLE
        self.issues = issues or []
        self.warnings = warnings or []
        self.confidence = confidence
        self.validation_time = validation_time
```

## Implementation Best Practices

### 1. Performance Optimization

When implementing cognitive planning, performance is critical for real-time operation:

```python
class OptimizedPlanningEngine(PlanningEngine):
    """
    Performance-optimized version of the planning engine.
    """
    
    def __init__(self, llm_config: dict):
        super().__init__(llm_config)
        
        # Initialize caches
        self.plan_cache = PlanCache(max_size=1000)
        self.context_cache = ContextCache(max_size=500)
        self.response_cache = ResponseCache(max_size=500)
        
        # Initialize async components
        self.execution_pool = ThreadPoolExecutor(max_workers=4)
        self.event_loop = asyncio.new_event_loop()
        
        # Performance monitoring
        self.metrics_collector = PlanningMetricsCollector()
    
    async def plan_task_async(self, 
                             command: str, 
                             context: Dict[str, Any], 
                             options: PlanningOptions = None) -> PlanningResult:
        """
        Asynchronously plan a task with performance optimizations.
        """
        start_time = time.time()
        
        # 1. Check plan cache first
        cache_key = self._create_cache_key(command, context, options)
        cached_result = self.plan_cache.get(cache_key)
        if cached_result:
            self.metrics_collector.record_cache_hit()
            return cached_result
        
        # 2. Use enriched context cache
        enriched_context = self.context_cache.get_or_compute(context, self.context_manager.enrich_context)
        
        # 3. Perform planning
        result = await self._plan_task_internal_async(command, enriched_context, options)
        
        # 4. Cache the result
        if result.success and result.confidence > 0.7:  # Only cache high-confidence results
            self.plan_cache.set(cache_key, result)
        
        # 5. Record metrics
        total_time = time.time() - start_time
        self.metrics_collector.record_planning_result(result, total_time)
        
        return result
    
    def _create_cache_key(self, command: str, context: Dict[str, Any], options: PlanningOptions) -> str:
        """
        Create a cache key for the planning request.
        """
        import hashlib
        
        context_subset = {k: v for k, v in context.items() 
                         if k not in ['timestamp', 'robot_state', 'perception']}  # Exclude volatile keys
        
        cache_data = {
            'command': command,
            'context': context_subset,
            'options': options.__dict__ if options else {}
        }
        
        cache_str = json.dumps(cache_data, sort_keys=True)
        return hashlib.md5(cache_str.encode()).hexdigest()
    
    async def _plan_task_internal_async(self, 
                                      command: str, 
                                      context: Dict[str, Any], 
                                      options: PlanningOptions) -> PlanningResult:
        """
        Internal planning method supporting async execution.
        """
        # This would use asyncio for non-blocking LLM calls
        loop = asyncio.get_running_loop()
        
        # Offload to thread pool for CPU-intensive operations
        result = await loop.run_in_executor(
            self.execution_pool,
            self._plan_task_blocking,
            command,
            context,
            options
        )
        
        return result
    
    def _plan_task_blocking(self, 
                           command: str, 
                           context: Dict[str, Any], 
                           options: PlanningOptions) -> PlanningResult:
        """
        Blocking planning method for thread pool execution.
        """
        # Call the parent's planning method
        return super().plan_task(command, context, options)

class PlanCache:
    """
    Cache for storing previously generated plans.
    """
    
    def __init__(self, max_size: int = 1000):
        self.capacity = max_size
        self.cache = {}
        self.access_order = deque()
        self.hit_count = 0
        self.miss_count = 0
    
    def get(self, key: str) -> Optional[PlanningResult]:
        """
        Get a cached plan result by key.
        """
        if key in self.cache:
            self.hit_count += 1
            
            # Move to end (most recently used)
            self.access_order.remove(key)
            self.access_order.append(key)
            
            return self.cache[key]
        else:
            self.miss_count += 1
            return None
    
    def set(self, key: str, value: PlanningResult):
        """
        Set a plan result in the cache.
        """
        if key in self.cache:
            # Update existing
            self.cache[key] = value
            self.access_order.remove(key)
            self.access_order.append(key)
        else:
            # Check if we need to evict
            if len(self.cache) >= self.capacity:
                # Remove least recently used
                lru_key = self.access_order.popleft()
                del self.cache[lru_key]
            
            # Add new item
            self.cache[key] = value
            self.access_order.append(key)
    
    def get_hit_rate(self) -> float:
        """
        Get the cache hit rate.
        """
        total = self.hit_count + self.miss_count
        return self.hit_count / total if total > 0 else 0.0
```

### 2. Error Handling and Recovery

Robust error handling is essential for reliable cognitive planning:

```python
class ResilientPlanningEngine(PlanningEngine):
    """
    Planning engine with enhanced error handling and recovery.
    """
    
    def __init__(self, llm_config: dict):
        super().__init__(llm_config)
        self.fallback_strategies = [
            self._simplify_command,
            self._use_template_based_planning,
            self._request_clarification
        ]
        self.retry_handler = RetryHandler()
    
    def plan_task_with_recovery(self, 
                               command: str, 
                               context: Dict[str, Any], 
                               options: PlanningOptions = None) -> PlanningResult:
        """
        Plan a task with error handling and recovery strategies.
        """
        try:
            # Primary planning attempt
            primary_result = self.plan_task(command, context, options)
            
            if primary_result.success:
                return primary_result
            
            self.logger.warning(f"Primary planning failed, attempting recovery")
            
            # Apply recovery strategies
            recovery_result = self._apply_recovery_strategies(
                command, context, options, primary_result
            )
            
            if recovery_result.success:
                self.logger.info("Planning succeeded with recovery")
                return recovery_result
            
            # If all recovery attempts fail, return the original failure
            self.logger.error("All planning and recovery attempts failed")
            return primary_result
            
        except Exception as e:
            self.logger.error(f"Unexpected error in planning with recovery: {str(e)}")
            return PlanningResult(
                success=False,
                error=f"Unexpected error: {str(e)}",
                confidence=0.0
            )
    
    def _apply_recovery_strategies(self, 
                                 command: str, 
                                 context: Dict[str, Any], 
                                 options: PlanningOptions, 
                                 original_result: PlanningResult) -> PlanningResult:
        """
        Apply recovery strategies in sequence.
        """
        for i, strategy in enumerate(self.fallback_strategies):
            try:
                self.logger.info(f"Trying recovery strategy {i+1}: {strategy.__name__}")
                
                recovery_result = strategy(command, context, options, original_result)
                
                if recovery_result and recovery_result.success:
                    self.logger.info(f"Recovery strategy {i+1} succeeded")
                    return recovery_result
                    
            except Exception as e:
                self.logger.warning(f"Recovery strategy {i+1} failed: {str(e)}")
                continue
        
        # All recovery strategies failed
        return original_result
    
    def _simplify_command(self, 
                         command: str, 
                         context: Dict[str, Any], 
                         options: PlanningOptions, 
                         error_result: PlanningResult) -> Optional[PlanningResult]:
        """
        Simplify the command and try planning again.
        """
        # Extract core intent from complex command
        simplified_command = self._extract_core_intent(command)
        
        if simplified_command and simplified_command != command:
            self.logger.info(f"Attempting simplified command: {simplified_command}")
            return self.plan_task(simplified_command, context, options)
        
        return None
    
    def _extract_core_intent(self, command: str) -> str:
        """
        Extract the core intent from a complex command.
        """
        # Remove secondary clauses
        # "Go to the kitchen and get me a cup" -> "Go to the kitchen"
        # "Pick up the red book on the table" -> "Pick up the book"
        
        # Look for primary action verbs
        action_patterns = [
            r'(go to|navigate to)\s+([^,]+)',  # navigate to location
            r'(pick up|grasp|take)\s+(the\s+)?([^,]+)',  # grasp object
            r'(move|go)\s+(forward|backward|left|right)',  # directional movement
        ]
        
        for pattern in action_patterns:
            match = re.search(pattern, command, re.IGNORECASE)
            if match:
                if len(match.groups()) > 1:
                    # For navigate to, just return the navigation part
                    if match.group(1).lower() in ['go to', 'navigate to']:
                        return f"{match.group(1)} {match.group(2)}"
                    # For grasp, return the grasp part
                    elif match.group(1).lower() in ['pick up', 'grasp', 'take']:
                        obj_part = match.group(3) if match.group(2) else match.group(2)
                        return f"{match.group(1)} the {obj_part}"
                    # For movement, return the movement part
                    else:
                        return f"{match.group(1)} {match.group(2)}"
        
        # If no simplification possible, return original
        return command
    
    def _use_template_based_planning(self, 
                                    command: str, 
                                    context: Dict[str, Any], 
                                    options: PlanningOptions, 
                                    error_result: PlanningResult) -> Optional[PlanningResult]:
        """
        Use template-based approach when LLM planning fails.
        """
        # Match command against known templates
        templates = self._get_planning_templates()
        
        for template in templates:
            if template.matches(command):
                try:
                    action_sequence = template.generate_actions(command, context)
                    return PlanningResult(
                        action_sequence=action_sequence,
                        reasoning=f"Generated using template: {template.name}",
                        confidence=0.6,  # Lower confidence for template-based
                        success=True
                    )
                except Exception as e:
                    self.logger.warning(f"Template {template.name} failed: {str(e)}")
                    continue
        
        return None
    
    def _get_planning_templates(self):
        """
        Get known planning templates.
        """
        return [
            NavigationTemplate(),
            GraspObjectTemplate(),
            PlaceObjectTemplate(),
            MoveBasedTemplate(),
            InspectObjectTemplate()
        ]
    
    def _request_clarification(self, 
                             command: str, 
                             context: Dict[str, Any], 
                             options: PlanningOptions, 
                             error_result: PlanningResult) -> Optional[PlanningResult]:
        """
        Generate a sequence to request clarification instead of failing completely.
        """
        # Create a special action to request user clarification
        clarify_action = ActionStep(
            action_type='request_clarification',
            parameters={
                'original_command': command,
                'confidence': error_result.confidence if hasattr(error_result, 'confidence') else 0.0,
                'issues': [str(error_result.error)] if hasattr(error_result, 'error') else []
            }
        )
        
        return PlanningResult(
            action_sequence=[clarify_action],
            reasoning="Original command unclear, requesting clarification",
            confidence=0.8,  # Confident in the clarification approach
            success=True
        )

class RetryHandler:
    """
    Handle retries with exponential backoff for flaky services.
    """
    
    def __init__(self, max_retries: int = 3, base_delay: float = 1.0, max_delay: float = 60.0):
        self.max_retries = max_retries
        self.base_delay = base_delay
        self.max_delay = max_delay
    
    def execute_with_retry(self, func, *args, **kwargs) -> Any:
        """
        Execute a function with retry logic.
        """
        last_exception = None
        
        for attempt in range(self.max_retries + 1):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                last_exception = e
                if attempt == self.max_retries:
                    # Final attempt failed
                    break
                
                # Calculate delay with exponential backoff
                delay = min(self.base_delay * (2 ** attempt), self.max_delay)
                self.logger.warning(f"Attempt {attempt + 1} failed: {str(e)}, retrying in {delay}s")
                time.sleep(delay)
        
        # If we get here, all retries failed
        raise last_exception

class PlanningTemplate:
    """
    Abstract base class for planning templates.
    """
    
    def __init__(self, name: str):
        self.name = name
    
    def matches(self, command: str) -> bool:
        """
        Check if this template matches the command.
        """
        raise NotImplementedError
    
    def generate_actions(self, command: str, context: Dict[str, Any]) -> List[ActionStep]:
        """
        Generate actions for the command using this template.
        """
        raise NotImplementedError

class NavigationTemplate(PlanningTemplate):
    """
    Template for navigation commands.
    """
    
    def __init__(self):
        super().__init__("Navigation")
        self.pattern = re.compile(r'(go to|navigate to|move to)\s+(.+)', re.IGNORECASE)
    
    def matches(self, command: str) -> bool:
        return bool(self.pattern.search(command))
    
    def generate_actions(self, command: str, context: Dict[str, Any]) -> List[ActionStep]:
        match = self.pattern.search(command)
        if not match:
            raise ValueError("Command doesn't match navigation pattern")
        
        destination = match.group(2)
        
        # Look up destination in context
        dest_info = self._lookup_destination(destination, context)
        
        if not dest_info:
            # Create a default navigation action
            return [ActionStep(
                action_type='navigate_to_location',
                parameters={'x': 1.0, 'y': 1.0, 'frame': 'map'},  # Default location
                estimated_duration=5.0
            )]
        
        return [ActionStep(
            action_type='navigate_to_location',
            parameters=dest_info,
            estimated_duration=5.0
        )]
    
    def _lookup_destination(self, destination: str, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Look up destination coordinates in context.
        """
        # Search for location in context
        locations = context.get('locations', {})
        for loc_name, loc_data in locations.items():
            if destination.lower() in loc_name.lower():
                return loc_data
        
        return None
```

### 3. Integration with ROS 2

The cognitive planning system integrates seamlessly with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer

from .planning_engine import ResilientPlanningEngine, PlanningOptions
from .context_manager import ContextManager
from .action_validator import ActionValidator


class CognitivePlanningNode(Node):
    """
    ROS 2 node for cognitive planning using LLMs.
    """
    
    def __init__(self):
        super().__init__('cognitive_planning_node')
        
        # Initialize planning components
        llm_config = {
            'model': self.declare_parameter('llm_model', 'gpt-3.5-turbo').value,
            'temperature': self.declare_parameter('llm_temperature', 0.3).value,
            'max_tokens': self.declare_parameter('llm_max_tokens', 1000).value,
            'timeout': self.declare_parameter('llm_timeout', 30.0).value
        }
        
        self.planning_engine = ResilientPlanningEngine(llm_config)
        self.context_manager = ContextManager()
        self.action_validator = ActionValidator()
        
        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        self.action_sequence_pub = self.create_publisher(
            String,
            'cognitive_action_sequence',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'cognitive_status',
            10
        )
        
        self.perception_sub = self.create_subscription(
            String,
            'environment_context',
            self.perception_callback,
            10
        )
        
        # Service server for direct planning requests
        self.plan_service = self.create_service(
            PlanCognitiveTask,
            'plan_cognitive_task',
            self.plan_cognitive_task_callback
        )
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info("Cognitive Planning Node initialized")
    
    def command_callback(self, msg: String):
        """
        Handle incoming natural language commands.
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        
        # Get current context
        current_context = self.context_manager.get_current_state()
        
        # Plan the task
        options = PlanningOptions(detailed=True, safety_required=True)
        result = self.planning_engine.plan_task_with_recovery(command, current_context, options)
        
        if result.success:
            # Publish the action sequence
            sequence_json = json.dumps({
                'action_sequence': [action.__dict__ for action in result.action_sequence],
                'reasoning': result.reasoning,
                'confidence': result.confidence
            })
            
            action_msg = String()
            action_msg.data = sequence_json
            self.action_sequence_pub.publish(action_msg)
            
            self.get_logger().info(f"Published action sequence with {len(result.action_sequence)} steps")
        else:
            self.get_logger().error(f"Planning failed: {result.error}")
            self.status_pub.publish(String(data=f"planning_failed: {result.error}"))
    
    def perception_callback(self, msg: String):
        """
        Update context with new perception data.
        """
        try:
            perception_data = json.loads(msg.data)
            self.context_manager.update_perception(perception_data)
            self.get_logger().debug("Updated context with new perception data")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse perception data: {str(e)}")
    
    def plan_cognitive_task_callback(self, request: PlanCognitiveTask.Request, 
                                   response: PlanCognitiveTask.Response) -> PlanCognitiveTask.Response:
        """
        Service callback for direct cognitive planning requests.
        """
        self.get_logger().info(f"Received planning request: {request.natural_language_command}")
        
        try:
            # Parse environment context
            context = json.loads(request.environment_context) if request.environment_context else {}
            
            # Add any additional context from the service request
            if request.robot_position:
                context['robot_position'] = {
                    'x': request.robot_position.position.x,
                    'y': request.robot_position.position.y,
                    'z': request.robot_position.position.z
                }
            
            # Plan the task
            options = PlanningOptions(
                detailed=request.detailed_planning,
                safety_required=request.safety_required
            )
            
            result = self.planning_engine.plan_task_with_recovery(
                request.natural_language_command, 
                context, 
                options
            )
            
            if result.success:
                response.action_sequence = json.dumps([
                    {**action.__dict__, 
                     'estimated_duration': getattr(action, 'estimated_duration', 1.0)}
                    for action in result.action_sequence
                ])
                response.reasoning = result.reasoning
                response.confidence = result.confidence
                response.success = True
                response.error_message = ""
                
                self.get_logger().info("Planning service completed successfully")
            else:
                response.success = False
                response.error_message = result.error or "Planning failed with unknown error"
                
                self.get_logger().error(f"Planning service failed: {response.error_message}")
        
        except Exception as e:
            response.success = False
            response.error_message = f"Service error: {str(e)}"
            self.get_logger().error(f"Planning service error: {str(e)}")
        
        return response
    
    def publish_status(self):
        """
        Publish periodic status updates.
        """
        status_msg = String()
        
        # Basic status - in a real system, you might include more detailed metrics
        status_msg.data = f"cognitive_planner_running"
        self.status_pub.publish(status_msg)

def main(args=None):
    """
    Main function to run the cognitive planning node.
    """
    rclpy.init(args=args)
    
    try:
        node = CognitivePlanningNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down cognitive planning node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Validation

### Unit Tests

```python
import unittest
from unittest.mock import Mock, patch, MagicMock
from cognitive_planning import PlanningEngine, PlanningOptions, PlanningResult


class TestPlanningEngine(unittest.TestCase):
    """
    Unit tests for the PlanningEngine class.
    """
    
    def setUp(self):
        """
        Set up test fixtures.
        """
        self.llm_config = {
            'model': 'gpt-3.5-turbo',
            'temperature': 0.3,
            'max_tokens': 1000,
            'timeout': 30
        }
        self.engine = PlanningEngine(self.llm_config)
    
    @patch('cognitive_planning.LLMClient')
    def test_plan_simple_navigation(self, mock_llm_client):
        """
        Test planning a simple navigation command.
        """
        # Mock LLM response
        mock_response = {
            'action_sequence': [
                {'action_type': 'navigate_to_location', 'parameters': {'x': 1.0, 'y': 2.0, 'frame': 'map'}}
            ],
            'reasoning': 'Navigate to the specified location.',
            'confidence': 0.85
        }
        mock_llm_client.return_value.chat_completion.return_value.choices = [
            Mock(message=Mock(content=json.dumps(mock_response)))
        ]
        
        # Test the planning
        command = "Go to the kitchen"
        context = {
            'robot_position': {'x': 0.0, 'y': 0.0},
            'locations': {'kitchen': {'x': 1.0, 'y': 2.0, 'frame': 'map'}}
        }
        
        result = self.engine.plan_task(command, context)
        
        # Assertions
        self.assertTrue(result.success)
        self.assertEqual(len(result.action_sequence), 1)
        self.assertEqual(result.action_sequence[0].action_type, 'navigate_to_location')
        self.assertGreaterEqual(result.confidence, 0.8)
    
    def test_preprocess_command(self):
        """
        Test command preprocessing.
        """
        # Test abbreviation expansion
        result = self.engine._preprocess_command("I wanna go to the kitchen")
        self.assertIn('want to', result)
        
        # Test normalization
        result = self.engine._preprocess_command("  GO TO THE KITCHEN  ")
        self.assertEqual(result, 'go to the kitchen')
    
    def test_create_planning_prompt(self):
        """
        Test planning prompt creation.
        """
        command = "Pick up the red cup"
        context = {'objects': [{'name': 'red cup', 'position': {'x': 1.0, 'y': 1.0}}]}
        options = PlanningOptions(detailed=True)
        
        prompt = self.engine._create_planning_prompt(command, context, options)
        
        # Check that important elements are in the prompt
        self.assertIn('red cup', prompt)
        self.assertIn('detailed', prompt)
        self.assertIn('grasp_object', prompt)
    
    def test_parse_llm_response(self):
        """
        Test LLM response parsing.
        """
        response = '''{
            "action_sequence": [
                {"action_type": "grasp_object", "parameters": {"object_name": "cup"}}
            ],
            "reasoning": "Grasp the cup",
            "confidence": 0.8
        }'''
        
        parsed = self.engine._parse_llm_response(response)
        
        self.assertEqual(len(parsed['action_sequence']), 1)
        self.assertEqual(parsed['action_sequence'][0]['action_type'], 'grasp_object')
        self.assertEqual(parsed['confidence'], 0.8)


class TestContextManager(unittest.TestCase):
    """
    Unit tests for ContextManager.
    """
    
    def setUp(self):
        self.context_manager = ContextManager()
    
    def test_enrich_context(self):
        """
        Test context enrichment.
        """
        base_context = {'objects': [{'name': 'table'}]}
        enriched = self.context_manager.enrich_context(base_context)
        
        # Should have additional fields
        self.assertIn('timestamp', enriched)
        self.assertIn('robot_state', enriched)
        self.assertIn('objects', enriched)
    
    def test_update_perception(self):
        """
        Test perception updates.
        """
        perception = {
            'type': 'object_detection',
            'object_id': 'obj1',
            'position': {'x': 1.0, 'y': 2.0}
        }
        
        self.context_manager.update_perception(perception)
        
        # Verify that the world state was updated
        current_state = self.context_manager.world_state.get_current_state()
        self.assertIn('obj1', current_state['objects'])


if __name__ == '__main__':
    unittest.main()
```

## Performance Monitoring and Optimization

### Metrics Collection

```python
class PlanningMetricsCollector:
    """
    Collect and report planning performance metrics.
    """
    
    def __init__(self):
        self.planning_times = []
        self.success_rates = []
        self.confidence_scores = []
        self.cache_hit_rates = []
        
        # Start metrics collection
        self.start_time = time.time()
    
    def record_planning_result(self, result: PlanningResult, planning_time: float):
        """
        Record metrics for a single planning operation.
        """
        self.planning_times.append(planning_time)
        self.success_rates.append(result.success)
        self.confidence_scores.append(result.confidence)
    
    def record_cache_hit(self):
        """
        Record a cache hit.
        """
        self.cache_hit_rates.append(True)
    
    def get_performance_report(self) -> str:
        """
        Generate a performance report.
        """
        n_ops = len(self.planning_times)
        if n_ops == 0:
            return "No planning operations recorded"
        
        avg_time = sum(self.planning_times) / len(self.planning_times)
        success_rate = sum(self.success_rates) / len(self.success_rates)
        avg_confidence = sum(self.confidence_scores) / len(self.confidence_scores) if self.confidence_scores else 0.0
        cache_hit_rate = sum(self.cache_hit_rates) / len(self.cache_hit_rates) if self.cache_hit_rates else 0.0
        
        uptime = time.time() - self.start_time
        
        report = f"""
        Cognitive Planning Performance Report
        ===================================
        Time Period: {uptime:.2f}s
        Operations: {n_ops}
        
        Performance:
        - Average Planning Time: {avg_time:.3f}s
        - Success Rate: {success_rate:.2%}
        - Average Confidence: {avg_confidence:.2f}
        - Cache Hit Rate: {cache_hit_rate:.2%}
        
        Percentiles (Planning Time):
        - 50th: {self._get_percentile(self.planning_times, 0.5):.3f}s
        - 90th: {self._get_percentile(self.planning_times, 0.9):.3f}s
        - 95th: {self._get_percentile(self.planning_times, 0.95):.3f}s
        """
        
        return report
    
    def _get_percentile(self, data: List[float], percentile: float) -> float:
        """
        Calculate percentile of data.
        """
        if not data:
            return 0.0
        
        sorted_data = sorted(data)
        index = int(percentile * len(sorted_data))
        return sorted_data[min(index, len(sorted_data) - 1)]
```

## Conclusion

This documentation provides comprehensive coverage of cognitive planning implementation in VLA systems, including:

1. **Architecture**: Detailed system architecture with all key components
2. **Implementation**: Complete code examples with error handling and performance optimizations
3. **Best Practices**: Performance optimization, error recovery, and ROS 2 integration
4. **Testing**: Unit tests to ensure reliability
5. **Monitoring**: Performance metrics and reporting

The cognitive planning system described here provides a robust foundation for converting natural language commands into executable robotic actions, with emphasis on safety, reliability, and performance. This implementation can be adapted for various robotic platforms and operational environments.