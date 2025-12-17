import React from 'react';

/**
 * ResponseFormatter Component
 * Formats agent responses for display in the chat interface
 */
const ResponseFormatter = ({ content, sources, confidence, timestamp }) => {
  // Format the response content with proper markdown support
  const formatContent = (rawContent) => {
    if (!rawContent) return '';

    // Replace markdown-like formatting with HTML elements
    let formattedContent = rawContent
      // Handle bold text
      .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
      // Handle italic text
      .replace(/\*(.*?)\*/g, '<em>$1</em>')
      // Handle code blocks
      .replace(/`(.*?)`/g, '<code>$1</code>')
      // Handle numbered lists
      .replace(/(\d+)\.\s(.+?)\n/g, '<ol><li>$2</li></ol>')
      // Handle bullet points
      .replace(/-\s(.+?)\n/g, '<ul><li>$1</li></ul>');

    return formattedContent;
  };

  // Format sources for display
  const formatSources = (sourceList) => {
    if (!sourceList || !Array.isArray(sourceList) || sourceList.length === 0) {
      return null;
    }

    return (
      <div className="message-sources">
        <strong>Sources:</strong>
        <ul>
          {sourceList.map((source, index) => (
            <li key={index}>{source}</li>
          ))}
        </ul>
      </div>
    );
  };

  // Format confidence level
  const formatConfidence = (conf) => {
    if (conf === undefined || conf === null) return null;

    const confidencePercentage = Math.round(conf * 100);
    let confidenceClass = 'message-confidence';

    // Add different classes based on confidence level
    if (confidencePercentage >= 80) {
      confidenceClass += ' high';
    } else if (confidencePercentage >= 60) {
      confidenceClass += ' medium';
    } else {
      confidenceClass += ' low';
    }

    return (
      <div className={confidenceClass}>
        <strong>Confidence:</strong> {confidencePercentage}%
      </div>
    );
  };

  // Format timestamp
  const formatTimestamp = (ts) => {
    if (!ts) return null;

    try {
      const date = new Date(ts);
      return (
        <div className="message-timestamp">
          {date.toLocaleDateString()} at {date.toLocaleTimeString([], {
            hour: '2-digit',
            minute: '2-digit'
          })}
        </div>
      );
    } catch (e) {
      console.warn('Invalid timestamp provided to ResponseFormatter:', ts);
      return null;
    }
  };

  return (
    <div className="formatted-response">
      <div
        className="formatted-content"
        dangerouslySetInnerHTML={{ __html: formatContent(content) }}
      />

      {formatConfidence(confidence)}

      {formatSources(sources)}

      {formatTimestamp(timestamp)}
    </div>
  );
};

export default ResponseFormatter;