# Reusable Docusaurus Components Guide

This guide describes how to create and use reusable components in the Docusaurus documentation for the digital twin module.

## Code Snippet Components

Docusaurus supports custom React components in Markdown files using MDX syntax. Here's how to create reusable code snippet components:

### Example: Terminal Command Component

Create a file `frontend/src/components/TerminalCommand/index.js`:

```jsx
import React from 'react';
import styles from './TerminalCommand.module.css';

const TerminalCommand = ({ command, output = "", comment = "" }) => {
  return (
    <div className={styles.terminalBlock}>
      <div className={styles.terminalHeader}>
        <span className={styles.terminalDot} style={{ backgroundColor: '#ff5f56' }}></span>
        <span className={styles.terminalDot} style={{ backgroundColor: '#ffbd2e' }}></span>
        <span className={styles.terminalDot} style={{ backgroundColor: '#27c93f' }}></span>
      </div>
      <div className={styles.terminalBody}>
        <div className={styles.commandLine}>
          <span className={styles.prompt}>$</span>
          <span className={styles.command}>{command}</span>
        </div>
        {output && (
          <div className={styles.output}>
            {output.split('\n').map((line, i) => (
              <div key={i}>{line}</div>
            ))}
          </div>
        )}
      </div>
      {comment && <div className={styles.comment}>{comment}</div>}
    </div>
  );
};

export default TerminalCommand;
```

And the corresponding CSS file `frontend/src/components/TerminalCommand/TerminalCommand.module.css`:

```css
.terminalBlock {
  margin: 1rem 0;
  border-radius: 8px;
  overflow: hidden;
  font-family: 'SF Mono', 'Monaco', 'Inconsolata', 'Fira Mono', 'Droid Sans Mono', 'Source Code Pro', monospace;
  font-size: 0.9rem;
  background-color: #2d2d2d;
  color: #f8f8f2;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.terminalHeader {
  display: flex;
  padding: 8px 12px;
  background: #1e1f29;
  align-items: center;
}

.terminalDot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  margin-right: 6px;
}

.terminalBody {
  padding: 12px;
}

.commandLine {
  display: flex;
  align-items: center;
  margin-bottom: 8px;
}

.prompt {
  color: #f92672;
  margin-right: 8px;
  user-select: none;
}

.command {
  color: #a6e22e;
  word-break: break-word;
}

.output {
  color: #f8f8f2;
  white-space: pre-wrap;
  line-height: 1.5;
}

.comment {
  padding: 8px 12px;
  background-color: #3a3a4a;
  color: #a9a9b3;
  font-size: 0.85rem;
  border-top: 1px solid #444;
}
```

### Using the Terminal Command Component

In your Markdown files, you can now use:

```md
import TerminalCommand from '@site/src/components/TerminalCommand';

<TerminalCommand 
  command="source /opt/ros/humble/setup.bash"
  output="ROS 2 Humble environment sourced"
  comment="Source the ROS 2 environment to use ROS 2 commands" />
```

## Technical Diagram Components

For technical diagrams, you can create React components that render SVG diagrams:

Create `frontend/src/components/ArchitectureDiagram/index.js`:

```jsx
import React from 'react';

const ArchitectureDiagram = ({ title, description }) => {
  return (
    <div style={{ textAlign: 'center', margin: '2rem 0' }}>
      <h3>{title}</h3>
      <div style={{ border: '1px solid #ccc', borderRadius: '8px', padding: '1rem', display: 'inline-block' }}>
        <svg width="600" height="300" xmlns="http://www.w3.org/2000/svg">
          {/* Gazebo Box */}
          <rect x="50" y="50" width="150" height="100" fill="#4A90E2" stroke="#333" strokeWidth="2" rx="10" />
          <text x="125" y="100" textAnchor="middle" fill="white" fontSize="16">Gazebo</text>
          <text x="125" y="125" textAnchor="middle" fill="white" fontSize="12">Physics</text>
          
          {/* ROS 2 Bridge */}
          <rect x="250" y="80" width="100" height="40" fill="#7ED321" stroke="#333" strokeWidth="2" rx="5" />
          <text x="300" y="105" textAnchor="middle" fill="white" fontSize="12">ROS 2</text>
          
          {/* Unity Box */}
          <rect x="400" y="50" width="150" height="100" fill="#BD10E0" stroke="#333" strokeWidth="2" rx="10" />
          <text x="475" y="100" textAnchor="middle" fill="white" fontSize="16">Unity</text>
          <text x="475" y="125" textAnchor="middle" fill="white" fontSize="12">Visualization</text>
          
          {/* Arrows */}
          <line x1="200" y1="100" x2="250" y2="100" stroke="#333" strokeWidth="2" markerEnd="url(#arrowhead)" />
          <line x1="350" y1="100" x2="400" y2="100" stroke="#333" strokeWidth="2" markerEnd="url(#arrowhead)" />
          
          {/* Arrowhead marker definition */}
          <defs>
            <marker id="arrowhead" markerWidth="10" markerHeight="7" 
              refX="9" refY="3.5" orient="auto">
              <polygon points="0 0, 10 3.5, 0 7" fill="#333" />
            </marker>
          </defs>
        </svg>
      </div>
      <p style={{ marginTop: '1rem', fontStyle: 'italic' }}>{description}</p>
    </div>
  );
};

export default ArchitectureDiagram;
```

### Using the Architecture Diagram Component

In your Markdown files:

```md
import ArchitectureDiagram from '@site/src/components/ArchitectureDiagram';

<ArchitectureDiagram 
  title="Digital Twin Architecture" 
  description="Shows the integration between Gazebo physics simulation and Unity visualization via ROS 2" />
```

## Best Practices for Reusable Components

1. **Keep components focused** - Each component should have a single, clear purpose
2. **Use descriptive names** - Make it obvious what the component does
3. **Make components configurable** - Use props to allow different variations
4. **Follow Docusaurus conventions** - Place components in `src/components` directory
5. **Style appropriately** - Use CSS modules to avoid style conflicts
6. **Document usage** - Include examples of how to use the component

These components can now be used throughout the digital twin module documentation to maintain consistency and improve the learning experience.