"""Apply the coordinate-speed-cap fix to pinned upstream gz-physics 7.8.0."""
from pathlib import Path
import sys
root=Path(sys.argv[1])
p=root/'bullet-featherstone/src/SDFFeatures.cc'
s=p.read_text();needle='jointInfo->maxVelocity = joint->Axis()->MaxVelocity();'
assert s.count(needle)==1
s=s.replace(needle,needle+'''
          // Honour high-speed rotor joints instead of Bullet's default 100 rad/s cap.
          if (std::isfinite(jointInfo->maxVelocity) &&
              jointInfo->maxVelocity > model->body->getMaxCoordinateVelocity())
            model->body->setMaxCoordinateVelocity(
                static_cast<btScalar>(jointInfo->maxVelocity));''')
p.write_text(s)
p=root/'bullet-featherstone/src/JointFeatures.cc';s=p.read_text();needle='jointInfo->maxVelocity = _value;';assert s.count(needle)==1
s=s.replace(needle,needle+'''
  const auto *model = this->ReferenceInterface<ModelInfo>(jointInfo->model);
  if (std::isfinite(_value) && _value > model->body->getMaxCoordinateVelocity())
    model->body->setMaxCoordinateVelocity(static_cast<btScalar>(_value));''');p.write_text(s)
# Match contact dimensions to the already inflated collision meshes, rather
# than adding an implicit 10 mm envelope to every mesh.
p=root/'bullet-featherstone/src/SDFFeatures.cc';s=p.read_text()
needle='this->meshesGImpact.back()->setMargin(btScalar(0.01));';assert s.count(needle)==1
s=s.replace(needle,'this->meshesGImpact.back()->setMargin(btScalar(0.0002));')
needle='''          // What is fdir1 for in the SDF's <bullet> spec?'''
assert s.count(needle)==1
# URDF conversion supplies ODE friction fields. Use them when Bullet fields
# are absent; the default all-rubber design is isotropic.
needle='''        if (const auto torsional = frictionElement->FindElement("torsional"))'''
assert s.count(needle)==1
s=s.replace(needle,'''        if (!frictionElement->FindElement("bullet"))
        {
          if (const auto ode = frictionElement->FindElement("ode"))
          {
            if (const auto f = ode->FindElement("mu")) mu = f->Get<double>();
            if (const auto f = ode->FindElement("mu2")) mu2 = f->Get<double>();
          }
        }
'''+needle)
# Bullet expects an effective length for spinning friction, while SDF stores
# a dimensionless coefficient and patch radius. The unmeasured default 3 mm
# patch is explicit, bounded, and included in the qualification assumptions.
needle='double torsionalCoefficient = 1.0;';assert s.count(needle)==1
s=s.replace(needle,needle+'\n  double patchRadius = 0.003;')
needle='''            torsionalCoefficient = coefficient->Get<double>();''';assert s.count(needle)==1
s=s.replace(needle,needle+'''
          if (const auto radius = torsional->FindElement("patch_radius"))
            if (radius->Get<double>() > 0.) patchRadius = radius->Get<double>();''')
needle='static_cast<btScalar>(torsionalCoefficient));';assert s.count(needle)==1
s=s.replace(needle,'static_cast<btScalar>(torsionalCoefficient * patchRadius));')
needle='''      linkInfo->collider->setAnisotropicFriction(
        btVector3(static_cast<btScalar>(mu), static_cast<btScalar>(mu2), 1),
        btCollisionObject::CF_ANISOTROPIC_FRICTION);''';assert s.count(needle)==1
s=s.replace(needle,'''      // RingQuad tyre fdir1 is its local axle Z; avoid multiplying mu twice.
      linkInfo->collider->setFriction(btScalar(1.0));
      linkInfo->collider->setAnisotropicFriction(
        btVector3(static_cast<btScalar>(mu2), static_cast<btScalar>(mu2),
                  static_cast<btScalar>(mu)),
        btCollisionObject::CF_ANISOTROPIC_FRICTION);''')
p.write_text(s)
