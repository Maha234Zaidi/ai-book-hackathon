# Citation Approach and Reference Formatting

This document outlines the citation approach and reference formatting for the Robotic Communication Systems module, following the research.md decisions and book constitution requirements.

## Citation Philosophy

### Source Verification Requirement
- 100% of factual claims must refer to credible sources (ROS docs, research papers, etc.)
- Prefer peer-reviewed robotics/AI research where needed
- All sources must be properly attributed to maintain academic integrity

### Citation Style
- Use IEEE format for all citations in the book
- Include both in-text citations and a comprehensive reference list at the end of each chapter

## In-Text Citation Format

### For Direct Quotes
- Use format: [Author, Year] or [Resource Name, Year] for documentation
- Example: "ROS 2 uses DDS as its middleware layer [ROS Documentation, 2023]"
- For direct quotes: "The key improvement in ROS 2 is the use of DDS" [ROS Documentation, 2023, p. 15]

### For Paraphrased Information
- Use the same format: [Author, Year] or [Resource Name, Year]
- Example: The use of DDS provides better scalability than previous solutions [ROS Documentation, 2023]

### For Data or Statistics
- Always provide source citation for any numerical data
- Example: ROS 2 supports up to 1000 nodes per system [ROS Performance Study, 2022]

## Reference List Format

Each chapter should end with a reference list using IEEE format:

[1] ROS.org, "ROS 2 Documentation: Overview and Architecture," 2023. [Online]. Available: https://docs.ros.org/en/humble/. [Accessed: Date].

[2] J. Doe, A. Smith, and B. Johnson, "Advancement in Robotic Middleware: A Comparative Study," in IEEE Transactions on Robotics, vol. 39, no. 2, pp. 123-135, 2023.

[3] NVIDIA, "Isaac Sim User Guide," NVIDIA Corporation, Santa Clara, CA, 2023.

## Acceptable Sources

### Primary Sources (Preferred)
- Official ROS 2 Documentation
- Primary research papers in robotics/AI
- Technical specifications from ROS 2 Working Group
- Academic textbooks on robotics

### Secondary Sources (Acceptable)
- Peer-reviewed articles referencing primary sources
- Technical blogs by recognized experts in the field
- Conference proceedings on robotics/AI

### Inadmissible Sources
- Unverified online tutorials
- Personal blogs without technical credentials
- Unpublished or pre-print research without peer review
- Sources that cannot be independently verified

## Documentation-Specific Citations

### ROS 2 Documentation
- Format: [ROS 2 Documentation, Version/Year, Topic/Section]
- Example: [ROS 2 Documentation, Humble, Nodes and Topics]

### Research Papers
- Format: [Author et al., Journal/Conference, Year, Page range]
- Example: [Smith et al., IEEE Transactions on Robotics, 2023, pp. 45-60]

### Standards and Specifications
- Format: [Standard Organization, Standard Number, Year]
- Example: [OMG, DDS Specification, 2020]

## Implementation Guidelines

### For Authors
- Always verify claims against official documentation before including them
- Maintain a working bibliography as you write each chapter
- Use consistent citation format throughout the module
- Include page numbers for direct quotes or specific information

### For Reviewers
- Verify that all factual claims have appropriate citations
- Check that citations follow the correct format
- Ensure sources are credible and verifiable
- Confirm that no claims are made without proper source attribution

## Example Citations in Context

This chapter references several key concepts that are further detailed in the official ROS 2 documentation [ROS 2 Documentation, Humble, Core Concepts]. The implementation of the publisher-subscriber pattern in ROS 2 has been praised for its flexibility and performance in real-world applications [Robotics Middleware Analysis, 2023].