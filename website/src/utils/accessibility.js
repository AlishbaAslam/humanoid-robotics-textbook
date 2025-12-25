/**
 * Accessibility utilities for the chat interface
 */

/**
 * Focus management utilities for keyboard navigation
 */
export const focusManagement = {
  /**
   * Focus the chat input field
   */
  focusInput: () => {
    const input = document.querySelector('.chat-input');
    if (input) {
      input.focus();
    }
  },

  /**
   * Focus the send button
   */
  focusSendButton: () => {
    const sendButton = document.querySelector('.chat-send-button');
    if (sendButton) {
      sendButton.focus();
    }
  },

  /**
   * Focus the first message in the chat
   */
  focusFirstMessage: () => {
    const firstMessage = document.querySelector('.message');
    if (firstMessage) {
      firstMessage.focus();
    }
  },

  /**
   * Focus the last message in the chat
   */
  focusLastMessage: () => {
    const messages = document.querySelectorAll('.message');
    if (messages.length > 0) {
      messages[messages.length - 1].focus();
    }
  }
};

/**
 * Keyboard navigation handlers
 */
export const keyboardNavigation = {
  /**
   * Handle keyboard shortcuts for the chat interface
   * @param {KeyboardEvent} event - The keyboard event
   */
  handleKeyboardShortcuts: (event) => {
    // Alt + K to focus chat input
    if (event.altKey && event.key === 'k') {
      event.preventDefault();
      focusManagement.focusInput();
    }

    // Alt + S to focus send button
    if (event.altKey && event.key === 's') {
      event.preventDefault();
      focusManagement.focusSendButton();
    }

    // Alt + M to focus first message
    if (event.altKey && event.key === 'm') {
      event.preventDefault();
      focusManagement.focusFirstMessage();
    }

    // Alt + L to focus last message
    if (event.altKey && event.key === 'l') {
      event.preventDefault();
      focusManagement.focusLastMessage();
    }

    // Escape key to clear focus
    if (event.key === 'Escape') {
      event.preventDefault();
      document.activeElement.blur();
    }
  },

  /**
   * Initialize keyboard navigation
   */
  init: () => {
    document.addEventListener('keydown', keyboardNavigation.handleKeyboardShortcuts);
  }
};

/**
 * Screen reader utilities
 */
export const screenReader = {
  /**
   * Announce a message to screen readers
   * @param {string} message - The message to announce
   */
  announce: (message) => {
    // Create a temporary element for screen readers
    const announcement = document.createElement('div');
    announcement.setAttribute('aria-live', 'polite');
    announcement.setAttribute('aria-atomic', 'true');
    announcement.className = 'sr-only';
    announcement.textContent = message;

    document.body.appendChild(announcement);

    // Remove after a delay to ensure it's read
    setTimeout(() => {
      document.body.removeChild(announcement);
    }, 1000);
  },

  /**
   * Hide elements from screen readers but keep them visually present
   * @param {HTMLElement} element - The element to hide from screen readers
   */
  hideFromScreenReader: (element) => {
    element.setAttribute('aria-hidden', 'true');
  },

  /**
   * Show elements to screen readers
   * @param {HTMLElement} element - The element to show to screen readers
   */
  showToScreenReader: (element) => {
    element.removeAttribute('aria-hidden');
  }
};

/**
 * ARIA attributes management
 */
export const aria = {
  /**
   * Set ARIA attributes for the chat interface
   */
  setChatAttributes: () => {
    const chatInterface = document.querySelector('.chat-interface');
    if (chatInterface) {
      chatInterface.setAttribute('role', 'region');
      chatInterface.setAttribute('aria-label', 'Humanoid Robotics Assistant Chat Interface');
    }

    const messagesContainer = document.querySelector('.chat-messages');
    if (messagesContainer) {
      messagesContainer.setAttribute('role', 'log');
      messagesContainer.setAttribute('aria-live', 'polite');
      messagesContainer.setAttribute('aria-label', 'Chat messages');
    }

    const input = document.querySelector('.chat-input');
    if (input) {
      input.setAttribute('aria-label', 'Type your question about humanoid robotics');
      input.setAttribute('aria-describedby', 'chat-input-help');
    }
  },

  /**
   * Update ARIA attributes when messages change
   */
  updateMessageAttributes: () => {
    const messages = document.querySelectorAll('.message');
    messages.forEach((message, index) => {
      message.setAttribute('role', 'listitem');
      message.setAttribute('aria-setsize', messages.length);
      message.setAttribute('aria-posinset', index + 1);
    });
  }
};

/**
 * Initialize all accessibility features
 */
export const initAccessibility = () => {
  // Set initial ARIA attributes
  aria.setChatAttributes();

  // Initialize keyboard navigation
  keyboardNavigation.init();

};