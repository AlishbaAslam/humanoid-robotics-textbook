# ADR-0001: OpenRouter Configuration for RAG System

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-23
- **Feature:** 004-frontend-backend-integration
- **Context:** The humanoid robotics textbook RAG system requires a reliable LLM provider that can handle context-rich queries with retrieved document chunks. Initially, the system used a mix of Gemini and OpenAI configurations, but needed to standardize on a single provider that offers good performance and cost-effectiveness for RAG queries.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **LLM Provider**: OpenRouter (with support for multiple models)
- **API Client**: OpenAI Python client library configured to work with OpenRouter's API
- **Base URL Configuration**: Configurable via environment variable (default: https://openrouter.ai/api/v1)
- **Model Selection**: Prioritize OPENROUTER_MODEL environment variable, fallback to agent_model setting
- **Error Handling**: Comprehensive error handling with fallback responses to ensure system reliability

## Consequences

### Positive

- Flexible model selection with support for multiple OpenRouter models
- Configurable base URL allows for potential API gateway or proxy usage
- Proper error handling prevents system crashes when API calls fail
- Backward compatibility maintained for Gemini configuration variables
- Cost-effective with access to free and premium models
- Standard OpenAI client library reduces complexity and maintenance

### Negative

- Dependency on external OpenRouter service for core functionality
- Potential API rate limits and costs with increased usage
- Possible vendor lock-in to OpenRouter's specific model endpoints
- Additional configuration complexity with multiple fallback mechanisms

## Alternatives Considered

**Alternative A: OpenAI API**
- Use standard OpenAI models directly (GPT-4, GPT-3.5-turbo)
- Why rejected: Higher cost, less model variety, no free tier for experimentation

**Alternative B: Self-hosted models (Ollama, Local LLMs)**
- Run models locally using Ollama or similar tools
- Why rejected: Higher computational requirements, less reliable performance, more infrastructure complexity

**Alternative C: Multiple LLM providers with fallback**
- Implement provider routing with fallback mechanisms (OpenAI → Anthropic → Cohere)
- Why rejected: Increased complexity, harder to maintain, over-engineering for current needs

## References

- Feature Spec: specs/004-frontend-backend-integration/spec.md
- Implementation Plan: specs/004-frontend-backend-integration/plan.md
- Related ADRs: none
- Evaluator Evidence: history/prompts/004-frontend-backend-integration/0001-fixed-openrouter-llm-configuration-for-rag-context.refactor.prompt.md
