# Global Writing Conventions for Robotic Communication Systems Module

This document defines the global writing conventions for the Robotic Communication Systems Education Module to ensure consistency across all chapters.

## Markdown Formatting

### Headings
- Use ATX-style headings with proper hierarchy
- H1 for chapter titles (only one per chapter)
- H2 for major sections
- H3 for subsections
- H4 for sub-subsections if needed
- Example:
  ```markdown
  # Chapter Title (H1)
  ## Major Section (H2)
  ### Subsection (H3)
  ```

### Code Blocks
- Always specify the language after the opening triple backticks
- Use Python examples for rclpy code as per research decisions
- Add line numbers only when specifically referring to lines
- Example:
  ```markdown
  ```python
  import rclpy
  from rclpy.node import Node
  
  class MinimalPublisher(Node):
      def __init__(self):
          super().__init__('minimal_publisher')
          self.publisher = self.create_publisher(String, 'topic', 10)
  ```
  ```

### Lists
- Use dash `-` for unordered lists
- Use numbers `1.` for ordered lists
- Maintain consistent indentation (2 spaces)

## Technical Terminology

### ROS 2 Concepts
- ROS 2 (not ros2 or ROS2)
- Node (capitalized when referring to ROS 2 nodes)
- Topic (capitalized when referring to ROS 2 topics)
- Service (capitalized when referring to ROS 2 services)
- Publisher/subscriber (lowercase when part of compound terms)
- rclpy (always lowercase)
- DDS (Data Distribution Service)

### Robotics Terms
- Humanoid robot (lowercase unless at beginning of sentence)
- URDF (Unified Robot Description Format)
- Links and joints (for URDF components)

## RAG-Friendly Content Structure

### Section Organization
- Each section should be self-contained with complete explanations
- Place definitions of new terms when first introduced
- Use clear, descriptive headings that can be used for content chunking

### Cross-References
- Use relative links between chapters in the same module
- Reference concepts by their full names in first use, then allow abbreviations

### Content Chunking
- Limit paragraphs to 4-6 sentences
- Break up long sections with H3 headings
- Include summary sentences at the end of major sections

## Python Code Examples

### Standards
- Follow PEP8 coding standards
- Include explanatory comments for complex code
- Focus on conceptual understanding, not implementation details
- Use pseudocode when necessary to illustrate concepts
- All examples should be conceptual and not require actual ROS installation

### Documentation
- Include brief comments explaining the purpose of code blocks
- Use docstrings where appropriate
- Add inline comments for non-obvious parts of ROS 2 code examples

## Educational Considerations

### Accessibility
- Define technical terms when first introduced
- Use analogies to familiar concepts (e.g., publishers/subscribers to message queues)
- Provide clear, step-by-step conceptual walkthroughs
- Consider students with Python knowledge but no robotics background

### Clarity
- Use clear, concise language
- Avoid jargon without explanation
- Maintain consistent terminology throughout
- Include practical applications that students can relate to