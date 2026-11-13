#include "../src/ringquad_description/plugins/FiniteLatch.hh"
#include <cassert>
#include <cmath>
int main() {
 ringquad::FiniteLatch a;
 assert(a.Update(true,0.,0.)==0.);
 assert(a.Update(true,.01,0.)<0. && !a.broken);
 assert(a.Update(false,.1,0.)==0. && !a.broken);
 assert(a.Update(true,.1,0.)==0. && a.broken);
 assert(a.Update(false,0.,0.)==0. && a.broken);
 assert(a.Update(true,0.,0.)==0. && a.broken);
 ringquad::FiniteLatch b;
 assert(b.Update(true,0.,100.)==0. && b.broken);
}
