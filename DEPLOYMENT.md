# Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics course book.

## Quick Deploy to Vercel (Recommended)

### Step 1: Prepare Repository

```bash
cd physical-ai-robotics-book
git init
git add .
git commit -m "Initial commit"
```

### Step 2: Push to GitHub

```bash
# Create repository on GitHub first, then:
git remote add origin https://github.com/yourusername/your-repo-name.git
git branch -M main
git push -u origin main
```

### Step 3: Deploy to Vercel

1. Go to [vercel.com](https://vercel.com)
2. Click "New Project"
3. Import your GitHub repository
4. Vercel will auto-detect Docusaurus
5. Click "Deploy"

Done! Your site will be live at `https://your-project.vercel.app`

## Deploy to GitHub Pages

### Step 1: Update Configuration

Edit `docusaurus.config.js`:

```js
url: 'https://yourusername.github.io',
baseUrl: '/your-repo-name/',
organizationName: 'yourusername',
projectName: 'your-repo-name',
```

### Step 2: Deploy

```bash
# Set your GitHub username
export GIT_USER=yourusername

# Deploy
npm run deploy
```

Your site will be at `https://yourusername.github.io/your-repo-name/`

## Deploy Chatbot Backend (Optional)

The chatbot backend can be deployed separately:

### Option 1: Vercel Serverless

1. Create `vercel.json` in chatbot directory
2. Deploy: `cd chatbot && vercel`

### Option 2: Railway

1. Create account at [railway.app](https://railway.app)
2. New Project → Deploy from GitHub
3. Set environment variables
4. Deploy

### Option 3: Render

1. Create account at [render.com](https://render.com)
2. New → Web Service
3. Connect GitHub repository
4. Set build command: `pip install -r requirements.txt`
5. Set start command: `cd api && python main.py`

### Environment Variables for Backend

Set these in your deployment platform:

```
OPENAI_API_KEY=sk-...
NEON_DATABASE_URL=postgres://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
```

### Update Frontend Config

After deploying backend, update `docusaurus.config.js`:

```js
customFields: {
  chatbotApiUrl: 'https://your-backend-url.com',
},
```

Redeploy frontend.

## Local Development

```bash
# Install dependencies
npm install

# Start dev server
npm start

# Build for production
npm run build

# Serve build locally
npm run serve
```

## Troubleshooting

### Build Fails on Vercel

**Issue**: "Module not found" errors  
**Solution**: Ensure all dependencies are in `package.json`

```bash
npm install --save-exact
```

### Chatbot Not Working

**Issue**: CORS errors  
**Solution**: Update CORS origins in `chatbot/api/main.py`:

```python
allow_origins=["https://your-frontend-domain.com"]
```

### Broken Links After Deployment

**Issue**: 404 on page navigation  
**Solution**: Check `baseUrl` in `docusaurus.config.js` matches your deployment path

## Performance Optimization

### 1. Enable Compression

Vercel and GitHub Pages automatically compress assets.

### 2. Optimize Images

```bash
# Install image optimization tools
npm install --save-dev imagemin imagemin-webp

# Compress images
npm run optimize-images
```

### 3. Lazy Load Components

Already configured in Docusaurus.

## Custom Domain

### Vercel

1. Go to Project Settings → Domains
2. Add your custom domain
3. Update DNS records as instructed

### GitHub Pages

1. Add `CNAME` file to `static/` directory:
   ```
   yourdomain.com
   ```

2. Configure DNS:
   ```
   Type: A
   Name: @
   Value: 185.199.108.153
   ```

## Monitoring

### Analytics

Add Google Analytics in `docusaurus.config.js`:

```js
themeConfig: {
  gtag: {
    trackingID: 'G-XXXXXXXXXX',
  },
}
```

### Error Tracking

Use Sentry for error monitoring:

```bash
npm install @sentry/react
```

## Backup

Regular backups are important:

```bash
# Backup to different branch
git checkout -b backup-$(date +%Y%m%d)
git push origin backup-$(date +%Y%m%d)
```

## Support

If you encounter issues:

1. Check [Docusaurus docs](https://docusaurus.io/)
2. Search [GitHub issues](https://github.com/facebook/docusaurus/issues)
3. Ask in [Discord](https://discord.gg/panaversity)

---

**Congratulations on deploying your course book!** 🎉
