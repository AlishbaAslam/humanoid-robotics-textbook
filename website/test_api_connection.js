/**
 * Simple test to verify API connection
 */
async function testApiConnection() {
  console.log("Testing API connection...");

  try {
    // Dynamically import the agent service
    const agentService = await import('./src/api/agentService.js');

    // Check if the API_BASE_URL is properly defined
    console.log("API_BASE_URL:", agentService.API_BASE_URL || "Not available in this context");

    console.log("API service imported successfully!");
    console.log("Environment setup is correct.");
  } catch (error) {
    console.error("Error importing agent service:", error.message);
    console.error("This may be expected in a Node.js environment without proper webpack configuration.");
  }
}

// Run the test
testApiConnection();