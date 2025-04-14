#include "ModelObject.h"

#include "raylib.h"

ModelObject::ModelObject(Model model, Vector3 position, float scale, Color tint, bool hasCollision)
    : GameObject(position, hasCollision), model(model), scale(scale), tint(tint) {}

void ModelObject::draw() const { DrawModel(model, position, scale, tint); }