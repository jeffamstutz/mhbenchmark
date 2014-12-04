
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of SURVICE Engineering Company nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <string>
#include <memory>

#include <ospray/ospray.h>

#include "core/Mesh.h"
#include "math/Math.h"

#include <omp.h>

using ato::core::Mesh;
using Material = ato::core::Mesh::Material;

using ato::math::Max;

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

#define STATUS_UPDATES 0
#define FRAME_COUNT    100

extern void save_image(int *pixels, int width, int height, string fname);


int main(int argc, const char *argv[])
{
  // Parse input arguments ////////////////////////////////////////////////////

  if (argc < 5)
  {
    cout << "Usage: benchmark [input model] [width] [height] "
         << "[output image file] {-vp x y z -vi x y z -vu x y z {-fv fov}}"
         << endl;
    return 1;
  }

  ospInit(&argc, argv);

  string inputModel  = argv[1];
  string width       = argv[2];
  string height      = argv[3];
  string outputImage = argv[4];

  int w = atoi(width.c_str());
  int h = atoi(height.c_str());

  float vp[3];
  float vi[3];
  float vu[3];
  float fov = argc > 17 ? atof(argv[18]) : 60.f;
  bool  defaultView = true;

  if (argc > 5)
  {
    vp[0] = atof(argv[6]);
    vp[1] = atof(argv[7]);
    vp[2] = atof(argv[8]);

    vi[0] = atof(argv[10]);
    vi[1] = atof(argv[11]);
    vi[2] = atof(argv[12]);

    vu[0] = atof(argv[14]);
    vu[1] = atof(argv[15]);
    vu[2] = atof(argv[16]);

    defaultView = false;
  }


  // Create the renderer //////////////////////////////////////////////////////

#if STATUS_UPDATES
  cout << "--> creating renderer" << endl;
#endif

  ospLoadModule("mhtk");

  OSPRenderer renderer = ospNewRenderer("xray");

  OSPCamera camera = ospNewCamera("perspective");
  ospCommit(camera);
  ospSetObject(renderer, "camera", camera);

  OSPFrameBuffer fb = ospNewFrameBuffer(osp::vec2i(w, h), OSP_RGBA_I8);


  // Load the model ///////////////////////////////////////////////////////////

  cout << "loading " << inputModel << endl;

  Mesh mesh;
  try
  {
    mesh.load(inputModel);
  }
  catch (const ato::core::Exception& error)
  {
    cerr << "Error:  " << error.what() << endl;
    return 1;
  }

#if STATUS_UPDATES
  cout << "--> creating OSPRay model" << endl;
#endif

  const uint ntris = mesh.triangles().size();

  OSPTriangleMesh ospmesh = ospNewTriangleMesh();

  // Vertex array

  vector<osp::vec3fa> vertices;
  for (auto &v : mesh.vertices())
    vertices.push_back(osp::vec3fa(v.m_p[0], v.m_p[1], v.m_p[2], 0.f));

  OSPData position = ospNewData(vertices.size(), OSP_FLOAT3A, vertices.data());
  ospSetData(ospmesh, "position", position);

  // Index and material ID arrays

  vector<osp::vec3i> tris;
  vector<uint>       matIds;
  for (auto &t : mesh.triangles())
  {
    tris.push_back(osp::vec3i(t.m_vID[0], t.m_vID[1], t.m_vID[2]));
    matIds.push_back(t.m_mID);
  }

  OSPData index = ospNewData(tris.size(), OSP_INT3, tris.data());
  ospSetData(ospmesh, "index", index);

  OSPData primMatID = ospNewData(ntris, OSP_UINT, matIds.data());
  ospSetData(ospmesh, "prim.materialID", primMatID);

  // Materials array

  vector<OSPMaterial> materialList;
  for (auto &m : mesh.materials())
  {
    OSPMaterial mat = ospNewMaterial(renderer, "OBJMaterial");
    ospSet3f(mat, "Ka",
             m.m_rgb[Material::Ambient][0],
             m.m_rgb[Material::Ambient][1],
             m.m_rgb[Material::Ambient][2]);
    ospSet3f(mat, "Kd",
             m.m_rgb[Material::Diffuse][0],
             m.m_rgb[Material::Diffuse][1],
             m.m_rgb[Material::Diffuse][2]);
    ospSet3f(mat, "Ks",
             m.m_rgb[Material::Specular][0],
             m.m_rgb[Material::Specular][1],
             m.m_rgb[Material::Specular][2]);
    ospCommit(mat);
    materialList.push_back(mat);
  }

  OSPData ospMaterialList = ospNewData(materialList.size(), OSP_OBJECT,
                                       &materialList[0]);
  ospSetData(ospmesh, "materialList", ospMaterialList);

  // Finish the mesh and add to ospray model

  ospCommit(ospmesh);

  OSPModel model = ospNewModel();
  ospAddGeometry(model, ospmesh);

  ospCommit(model);

  ospSetObject(renderer, "world", model);
  ospSetObject(renderer, "model", model);

  ospCommit(renderer);


  // Setup camera /////////////////////////////////////////////////////////////

#if STATUS_UPDATES
  cout << "--> setting up camera" << endl;
#endif

  auto bounds  = mesh.bounds();
  auto diag    = bounds.diagonal();
  auto lookat  = bounds.center();
  auto maxdist = Max(fabs(diag[0]), fabs(diag[1]), fabs(diag[2]));

  osp::vec3f E(lookat[0], lookat[1] - (1.25f * maxdist), lookat[2]);
  osp::vec3f L(lookat[0]-E[0], lookat[1]-E[1], lookat[2]-E[2]);
  osp::vec3f U(0.f, 0.f, 1.f);

  if (!defaultView)
  {
    E = osp::vec3f(vp[0], vp[1], vp[2]);
    L = osp::vec3f(vi[0]-vp[0], vi[1]-vp[1], vi[2]-vp[2]);
    U = osp::vec3f(vu[0], vu[1], vu[2]);
  }

  ospSetVec3f(camera, "pos", E);
  ospSetVec3f(camera, "dir", L);
  ospSetVec3f(camera, "up",  U);
  ospSetf(  camera, "fovy", fov);
  ospSetf(  camera, "aspect", static_cast<float>(w)/h);
  ospCommit(camera);


  // Trace frames /////////////////////////////////////////////////////////////

  cout << endl << "tracing " << FRAME_COUNT << " frames..." << endl;

  double start = omp_get_wtime();

  for (auto i = 0; i < FRAME_COUNT; ++i)
    ospRenderFrame(fb, renderer);

  double end  = omp_get_wtime();
  double time = end - start;
  double fps  = FRAME_COUNT/time;

  cout << "...finished in " << time << "s" << endl;
  cout << endl;
  cout << " FPS: " << fps << endl;
  cout << "MRPS: " << (w*h*fps)/(1024*1024) << endl;


  // Save the image to a file /////////////////////////////////////////////////

#if STATUS_UPDATES
  cout << endl << "--> saving image: " << outputImage << endl;
#endif

  int *pixels = (int*)ospMapFrameBuffer(fb);

  save_image(pixels, w, h, outputImage);

  ospUnmapFrameBuffer(pixels, fb);


  // Cleanup //////////////////////////////////////////////////////////////////

#if STATUS_UPDATES
  cout << "--> cleaning up" << endl;
#endif

  ospRelease(renderer);

  return 0;
}
