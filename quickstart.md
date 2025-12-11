# Deployment Guide for Physical AI & Humanoid Robotics Textbook Platform

This guide provides instructions for deploying the Physical AI & Humanoid Robotics Textbook Platform to production environments.

## Architecture Overview

The platform consists of:
- **Frontend**: Docusaurus-based documentation site
- **Backend**: FastAPI application with RAG capabilities
- **Vector Database**: Qdrant for textbook content storage
- **Relational Database**: NeonDB (PostgreSQL) for user data and personalization

## Prerequisites

- Node.js 18+ for frontend
- Python 3.9+ for backend
- Access to Qdrant Cloud or self-hosted Qdrant instance
- NeonDB PostgreSQL database access
- OpenAI API key
- Domain name and SSL certificate

## Backend Deployment

### Environment Configuration

Create a `.env` file with the following required variables:

```env
# Server Settings
HOST=0.0.0.0
PORT=8000
DEBUG=False

# Database Settings
NEON_DB_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require
QDRANT_URL=https://your-cluster-url.qdrant.tech:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# OpenAI Settings
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-4-turbo

# Better-Auth Settings
AUTH_SECRET=generate-a-secure-jwt-secret-key-here
AUTH_ALGORITHM=HS256
AUTH_ACCESS_TOKEN_EXPIRE_MINUTES=30
AUTH_REFRESH_TOKEN_EXPIRE_DAYS=7

# RAG Settings
RAG_MAX_RESPONSE_TIME_MS=800
RAG_CONFIDENCE_THRESHOLD=0.7
RAG_TOP_K_RESULTS=5

# Translation Settings
URDU_TRANSLATION_ENABLED=True

# Security Settings
ALLOWED_ORIGINS=https://yourdomain.com,https://www.yourdomain.com
```

### Backend Deployment Steps

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd my-Physical-AI-Humanoid-Robotics-Book/backend
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Set up environment variables** (as shown above)

4. **Run database migrations** (if applicable):
   ```bash
   # This would depend on your specific migration system
   python -m src.database init
   ```

5. **Start the application**:
   ```bash
   uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --workers 4
   ```

## Frontend Deployment

### Docusaurus Deployment

1. **Navigate to frontend directory**:
   ```bash
   cd frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Build the site**:
   ```bash
   npm run build
   ```

4. **Serve or deploy**:
   - For GitHub Pages: Use GitHub Actions or manual deployment
   - For Vercel: `vercel --prod`
   - For Netlify: Use Netlify dashboard or CLI

## Database Setup

### NeonDB Configuration

1. Create a NeonDB project
2. Note the connection string
3. Update the `NEON_DB_URL` in your environment variables

### Qdrant Configuration

1. Set up Qdrant Cloud or self-hosted instance
2. Obtain the API key and URL
3. Update `QDRANT_URL` and `QDRANT_API_KEY` in environment variables

## Content Ingestion

After deployment, you need to ingest the textbook content into Qdrant:

1. Run the content ingestion script:
   ```bash
   cd backend
   python -m src.services.content_ingestion
   ```

## API Endpoints

Key API endpoints:
- `GET /health` - Health check
- `POST /rag/query` - RAG query endpoint
- `POST /personalize/` - Store user background
- `POST /translate-ur/` - Urdu translation endpoint
- `GET /docs` - API documentation

## Environment-Specific Configurations

### Production Environment
- Set `DEBUG=False`
- Use strong `AUTH_SECRET`
- Configure proper `ALLOWED_ORIGINS`
- Set appropriate resource limits

### Staging Environment
- Similar to production but with test databases
- Use staging-specific API keys
- Enable additional logging if needed

## Monitoring and Logging

The application uses standard logging. Configure your deployment platform to:
- Collect application logs
- Monitor response times
- Track error rates
- Set up alerts for critical issues

## Scaling Recommendations

- **Backend**: Use multiple workers based on CPU count
- **Database**: Scale NeonDB based on read/write patterns
- **Qdrant**: Scale based on query volume and data size
- **Frontend**: Use CDN for static asset delivery

## Security Considerations

- Use HTTPS for all traffic
- Rotate API keys regularly
- Monitor for unusual access patterns
- Implement rate limiting as needed
- Keep dependencies updated

## Troubleshooting

### Common Issues

1. **Database Connection Errors**: Verify connection strings and network access
2. **API Key Issues**: Check OpenAI and Qdrant API keys
3. **CORS Errors**: Verify `ALLOWED_ORIGINS` setting
4. **Performance Issues**: Check database and vector search performance

### Health Checks

Use the `/health` endpoint to verify service status:
```bash
curl https://your-api-domain/health
```

## Rollback Procedure

1. Keep previous deployment artifacts
2. Maintain database compatibility between versions
3. Use environment variables for feature flags when possible
4. Test rollback procedures in staging environment

## Maintenance Tasks

- Regular dependency updates
- Database backups
- Vector database optimization
- Content updates and re-ingestion
- Performance monitoring and tuning