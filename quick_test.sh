#!/bin/bash

# Quick test script for ragdoll functionality
echo "🚀 Quick Ragdoll Test"
echo "===================="

cd build

echo "📦 Building project..."
make -j4

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "🎮 Starting game (press ESC to exit)..."
    echo ""
    echo "Test checklist:"
    echo "✓ Character should stand upright immediately"
    echo "✓ Press SPACE to jump - should work!"
    echo "✓ Use WASD to move around"
    echo "✓ Character should remain stable"
    echo ""
    ./GameProject
else
    echo "❌ Build failed!"
    exit 1
fi
