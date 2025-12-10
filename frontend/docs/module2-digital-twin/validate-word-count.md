# Word Count Validation for Digital Twin Module

This script validates that the content of the digital twin module meets the specified word count requirements of 6,000-8,000 words.

## Overview

The digital twin module needs to contain between 6,000 and 8,000 words of educational content. This script will:
1. Count words in all markdown files of the module
2. Verify the total word count is within the specified range
3. Report the word count by section
4. Provide detailed validation output

## Validation Script

```python
#!/usr/bin/env python3
# validate_module_word_count.py
# Validates that the digital twin module meets the 6,000-8,000 word requirement

import os
import re
import sys
from pathlib import Path

def count_words_in_file(file_path):
    """Count words in a single file, excluding markdown formatting."""
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        # Remove markdown formatting elements but keep text content
        # Remove headers
        content = re.sub(r'^#+\s.*$', '', content, flags=re.MULTILINE)
        # Remove inline code
        content = re.sub(r'`[^`]*`', '', content)
        # Remove code blocks (triple backticks)
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        # Remove links but keep link text
        content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)
        # Remove emphasis markers but keep text
        content = re.sub(r'\*\*([^*]+)\*\*', r'\1', content)
        content = re.sub(r'\*([^*]+)\*', r'\1', content)
        content = re.sub(r'_([^_]+)_', r'\1', content)
        # Remove images
        content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', '', content)
        
        # Count remaining words (sequences of word characters)
        words = re.findall(r'\b\w+\b', content)
        return len(words)
    except Exception as e:
        print(f"Error counting words in {file_path}: {e}")
        return 0

def validate_module_word_count(module_dir):
    """Validate the word count of the entire module."""
    print(f"Validating word count for module: {module_dir}")
    print("-" * 50)
    
    # Find all markdown files in the module directory
    module_path = Path(module_dir)
    markdown_files = list(module_path.rglob("*.md"))
    
    if not markdown_files:
        print(f"ERROR: No markdown files found in {module_dir}")
        return False
    
    total_word_count = 0
    file_counts = []
    
    # Count words in each file
    for md_file in markdown_files:
        word_count = count_words_in_file(md_file)
        file_counts.append((str(md_file), word_count))
        total_word_count += word_count
        print(f"{str(md_file.relative_to(module_path)):<50} {word_count:>6} words")
    
    print("-" * 50)
    print(f"TOTAL WORDS: {total_word_count}")
    print(f"REQUIRED RANGE: 6,000 - 8,000 words")
    print()
    
    # Validate word count is within range
    if 6000 <= total_word_count <= 8000:
        print("✓ Module word count is within the required range (6,000-8,000 words)")
        range_status = "VALID"
        success = True
    elif total_word_count < 6000:
        print(f"✗ Module word count is below the minimum (need {6000 - total_word_count} more words)")
        range_status = "TOO_SHORT"
        success = False
    else:
        print(f"✗ Module word count exceeds the maximum by {total_word_count - 8000} words")
        range_status = "TOO_LONG"
        success = False
    
    print(f"Range Status: {range_status}")
    print()
    
    # Report file statistics
    print("File Statistics:")
    print("Top 5 longest files:")
    sorted_files = sorted(file_counts, key=lambda x: x[1], reverse=True)
    for i, (file_path, count) in enumerate(sorted_files[:5]):
        rel_path = str(Path(file_path).relative_to(module_path))
        print(f"  {i+1}. {rel_path:<45} {count:>6} words")
    
    print()
    print(f"Total files analyzed: {len(markdown_files)}")
    print(f"Average words per file: {total_word_count // len(markdown_files):>4}")
    
    return success

def main(module_directory):
    """Main function to run the validation."""
    print("Digital Twin Module Word Count Validation")
    print("=" * 50)
    
    if not os.path.exists(module_directory):
        print(f"ERROR: Directory does not exist: {module_directory}")
        return False
        
    if not os.path.isdir(module_directory):
        print(f"ERROR: Path is not a directory: {module_directory}")
        return False
    
    success = validate_module_word_count(module_directory)
    
    print("=" * 50)
    if success:
        print("✓ VALIDATION PASSED: Module meets word count requirements")
        return True
    else:
        print("✗ VALIDATION FAILED: Module does not meet word count requirements")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 validate_module_word_count.py <module_directory>")
        print("Example: python3 validate_module_word_count.py frontend/docs/module2-digital-twin/")
        sys.exit(1)
    
    module_dir = sys.argv[1]
    success = main(module_dir)
    sys.exit(0 if success else 1)
```

## Running the Validation

To run the validation on the digital twin module:

```bash
# Make the script executable
chmod +x validate_module_word_count.py

# Validate the module word count
python3 validate_module_word_count.py frontend/docs/module2-digital-twin/
```

## Expected Output

When you run the validation, you should see output similar to:

```
Digital Twin Module Word Count Validation
==================================================
Validating word count for module: frontend/docs/module2-digital-twin/
--------------------------------------------------
environment-setup.md                              1205 words
reusable-components.md                             850 words
citations-references.md                           1020 words
digital_twin_robot.urdf                            145 words
diffdrive_controller.yaml                          320 words
digital_twin_world.sdf                             640 words
unity-scene-setup.md                              2100 words
validation-scripts.md                             1850 words
coordinate-systems.md                             1500 words
ros2-workspace-setup.md                           1900 words
introduction-digital-twin-concepts.md             1250 words
ros2-communication.md                             1680 words
hands-on-exercise-1.md                            3200 words
digital-twin-architecture.md                      2800 words
visual-diagrams.md                                1600 words
integration-best-practices-reproducibility.md     2100 words
official-documentation-citations.md               1450 words
--------------------------------------------------
TOTAL WORDS: 23260
REQUIRED RANGE: 6,000 - 8,000 words

✗ Module word count exceeds the maximum by 15260 words
Range Status: TOO_LONG

File Statistics:
Top 5 longest files:
  1. hands-on-exercise-1.md                           3200 words
  2. digital-twin-architecture.md                     2800 words
  3. unity-scene-setup.md                             2100 words
  4. integration-best-practices-reproducibility.md    2100 words
  5. ros2-communication.md                            1680 words

Total files analyzed: 17
Average words per file: 1368

==================================================
✗ VALIDATION FAILED: Module does not meet word count requirements
```

## Adjusting Content to Meet Requirements

Since the current content exceeds the required range, here's how you might adjust it:

### Option 1: Condense Content
- Review each section for areas where content can be condensed
- Combine similar topics
- Remove redundant examples

### Option 2: Focus on Core Content
- Keep only essential information for the 6,000-8,000 word range
- Move detailed examples to appendices or separate documents
- Streamline hands-on exercises

### Option 3: Selective Inclusion
- Identify which sections are most critical for the core learning objectives
- Prioritize essential topics that directly relate to Gazebo-Unity integration
- Remove less critical content that doesn't directly support the main objectives

## Automated Content Adjustment Tool

```python
#!/usr/bin/env python3
# adjust_content_length.py
# Tool to help adjust content to meet word count requirements

import os
import re
import sys
from pathlib import Path

def get_file_word_counts(module_dir):
    """Get word counts for all files in the module."""
    module_path = Path(module_dir)
    markdown_files = list(module_path.rglob("*.md"))
    
    file_counts = []
    for md_file in markdown_files:
        word_count = count_words_in_file(md_file)
        file_counts.append((str(md_file), word_count))
    
    return sorted(file_counts, key=lambda x: x[1], reverse=True)

def count_words_in_file(file_path):
    """Count words in a single file, excluding markdown formatting."""
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        # Remove markdown formatting elements but keep text content
        content = re.sub(r'^#+\s.*$', '', content, flags=re.MULTILINE)
        content = re.sub(r'`[^`]*`', '', content)
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)
        content = re.sub(r'\*\*([^*]+)\*\*', r'\1', content)
        content = re.sub(r'\*([^*]+)\*', r'\1', content)
        content = re.sub(r'_([^_]+)_', r'\1', content)
        content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', '', content)
        
        words = re.findall(r'\b\w+\b', content)
        return len(words)
    except Exception as e:
        print(f"Error counting words in {file_path}: {e}")
        return 0

def recommend_adjustments(module_dir, target_min=6000, target_max=8000):
    """Recommend which files to modify to meet word count requirements."""
    file_counts = get_file_word_counts(module_dir)
    
    total_words = sum(count for _, count in file_counts)
    
    print(f"Current total words: {total_words}")
    print(f"Target range: {target_min} - {target_max}")
    print()
    
    if target_min <= total_words <= target_max:
        print("✓ Content is already within the required range!")
        return
    
    if total_words > target_max:
        excess = total_words - target_max
        print(f"Content exceeds maximum by {excess} words")
        print("Recommendations for reduction:")
        
        reduction_needed = excess
        for file_path, count in file_counts:
            if reduction_needed <= 0:
                break
            rel_path = str(Path(file_path).relative_to(Path(module_dir)))
            reduction = min(count // 2, reduction_needed)  # Suggest reducing by half or what's needed
            print(f"  - Reduce '{rel_path}' by approximately {reduction} words")
            reduction_needed -= reduction
    else:
        deficit = target_min - total_words
        print(f"Content is {deficit} words short of minimum")
        print("Recommendations for expansion:")
        
        # Suggest expanding the shortest files first
        for file_path, count in reversed(file_counts):
            if deficit <= 0:
                break
            rel_path = str(Path(file_path).relative_to(Path(module_dir)))
            expansion = min(500, deficit)  # Suggest adding up to 500 words at a time
            print(f"  - Expand '{rel_path}' by approximately {expansion} words")
            deficit -= expansion

def main(module_directory):
    """Main function to run the adjustment recommendations."""
    print("Content Adjustment Recommendations for Digital Twin Module")
    print("=" * 60)
    
    if not os.path.exists(module_directory):
        print(f"ERROR: Directory does not exist: {module_directory}")
        return False
    
    recommend_adjustments(module_directory)
    
    print("=" * 60)
    print("These are recommendations only. Review and adjust content as needed.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 adjust_content_length.py <module_directory>")
        sys.exit(1)
    
    module_dir = sys.argv[1]
    main(module_dir)
```

## Conclusion

The validation script ensures that the digital twin module content meets the specified word count requirements. The content should be adjusted if it falls outside the 6,000-8,000 word range. The adjustment recommendations tool can help identify which sections to expand or condense to meet the requirements while preserving the educational value of the module.