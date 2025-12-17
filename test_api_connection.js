/**
 * Test script to verify API connection and error handling
 */

// Test API connection
async function testApiConnection() {
  console.log('Testing API connection...');

  try {
    // Test health check endpoint
    const healthResponse = await fetch('http://localhost:8000/health');
    const healthData = await healthResponse.json();
    console.log('Health check response:', healthData);

    if (healthResponse.ok) {
      console.log('✅ Health check successful');
    } else {
      console.log('❌ Health check failed:', healthResponse.status, healthData);
    }
  } catch (error) {
    console.log('❌ Network error during health check:', error.message);
    console.log('This could be because the backend server is not running on http://localhost:8000');
  }

  // Test CORS by attempting to make a cross-origin request
  console.log('\nTesting CORS functionality...');
  try {
    // This will test if CORS is properly configured
    const corsTest = await fetch('http://localhost:8000/health', {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        'X-Requested-With': 'XMLHttpRequest' // Common header that might trigger CORS
      }
    });

    console.log('CORS test response status:', corsTest.status);

    // Check for CORS headers in response (if available)
    const corsHeaders = [...corsTest.headers.entries()].filter(([header]) =>
      header.toLowerCase().includes('access-control')
    );
    console.log('CORS headers found:', corsHeaders);

  } catch (error) {
    console.log('❌ CORS test error:', error.message);
  }

  console.log('\nTest completed. If the backend is not running, please start it with:');
  console.log('cd backend && python -m uvicorn src.api.main:app --reload --port 8000');
}

// Run the test
testApiConnection();