
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012-2014, Christiaan Gribble <cgribble[]rtvtk org>
 * Copyright (c) 2013, Ethan Kerzner <ethan.kerzner[]gmail org>
 * Copyright (c) 2012, Grove City College
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
 *  * Neither the name of Grove City College nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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


#include <cfloat>
#include <cstring>

#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

#include <core/io/BufferedReader.t>
#include <core/io/BufferedWriter.t>

#include <core/Mesh.h>
#include <core/RGB.t>
#include <core/Texture.h>
// #include <core/Timer.h>
#include <core/Utility.h>

// XXX(cpg) - temporary
#include <core/io/StreamIO.h>


/*! @bug XXX(cpg) - in general, an Error during geometry/material load doesn't
 *                  free resources correctly; fix me!
 */

/*! @todo TODO(cpg) - replace the many local ostringstream objects with an
 *                    inline to_string function (begs re-introduction of
 *                    Utility.h from vissim codebase)
 */

#ifdef WIN32
#  undef max
#endif // WIN32

#define USE_IFSTREAM


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::fstream;
    using std::ostream;
    using std::numeric_limits;
    using std::ostringstream;
    using std::pair;

    using core::io::BufferedReader;
    using core::io::BufferedWriter;

    using core::matchDirectory;
    using core::to_string;


    ///////////////////////////////////////////////////////////////////////////
    // Mesh member definitions

    // XXX(cpg) - where should this go?
#if defined(WIN32) && defined(RGB)
#  undef RGB
#endif // defined(WIN32) && defined(RGB)

    const core::Float::RGB Mesh::DefaultColor[] =
    {
      core::Float::RGB(0.1f, 0.1f, 0.1f),  // ambient
      core::Float::RGB(0.5f, 0.5f, 0.5f),  // diffuse
      core::Float::RGB::Zero,              // specular
      core::Float::RGB::Zero,              // transmissive
      core::Float::RGB::Zero,              // emissive
      core::Float::RGB::Invalid            // unused
    };

    const int Mesh::Vertex::Undefined = 0x7ffffff0;

    Mesh::Material::Material(const string& name) :
      m_name(name),
      m_ns(0.f),
      m_ni(0.f),
      m_tr(0.f)
    {
      for (uint i = 0; i < ntypes; ++i)
      {
        m_rgb[i]      = Mesh::DefaultColor[i];
        m_filename[i] = "";
        m_tex[i]      = 0;
        m_uscale[i]   = 0.f;
        m_vscale[i]   = 0.f;
      }
    }

    bool Mesh::Material::read(istream& in)
    {
      // Read name
      uint len;
      char name[1024];

      in.read(reinterpret_cast<char*>(&len), sizeof(uint));
      len = (len > 1024 ? 1024 : len);

      in.read(reinterpret_cast<char*>(name), len*sizeof(char));

      m_name = name;

      for (uint i = 0; i < ntypes; ++i)
      {
        // Read colors
        in.read(reinterpret_cast<char*>(&m_rgb[i][0]), sizeof(float));
        in.read(reinterpret_cast<char*>(&m_rgb[i][1]), sizeof(float));
        in.read(reinterpret_cast<char*>(&m_rgb[i][2]), sizeof(float));

        // Read textures
        bool valid;
        in.read(reinterpret_cast<char*>(&valid), sizeof(bool));
        if (valid)
        {
          m_tex[i] = new Texture();
          m_tex[i]->read(in);
        }

        // Read uscale, vscale
        in.read(reinterpret_cast<char*>(&m_uscale[i]), sizeof(float));
        in.read(reinterpret_cast<char*>(&m_vscale[i]), sizeof(float));
      }

    // Read exp, r0, tr
    in.read(reinterpret_cast<char*>(&m_ns), sizeof(float));
    in.read(reinterpret_cast<char*>(&m_ni), sizeof(float));
    in.read(reinterpret_cast<char*>(&m_tr), sizeof(float));

    return in.good();
  }

    bool Mesh::Material::write(ostream& out) const
    {
      // Write name
            uint  len  = m_name.length();
      const char* name = m_name.c_str();

      out.write(reinterpret_cast<const char*>(&len ),     sizeof(uint));
      out.write(reinterpret_cast<const char*>(&name), len*sizeof(char));

      for (uint i = 0; i < ntypes; ++i)
      {
        // Write colors
        out.write(reinterpret_cast<const char*>(&m_rgb[i][0]), sizeof(float));
        out.write(reinterpret_cast<const char*>(&m_rgb[i][1]), sizeof(float));
        out.write(reinterpret_cast<const char*>(&m_rgb[i][2]), sizeof(float));

        // Write textures
        if (m_tex[i])
        {
          bool valid = true;
          out.write(reinterpret_cast<const char*>(&valid), sizeof(bool));
          m_tex[i]->write(out);
        }

        // Write uscale, vscale
        out.write(reinterpret_cast<const char*>(&m_uscale[i]), sizeof(float));
        out.write(reinterpret_cast<const char*>(&m_vscale[i]), sizeof(float));
      }

      // Write exp, r0, tr
      out.write(reinterpret_cast<const char*>(&m_ns), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_ni), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_tr), sizeof(float));

      return out.good();
    }

    Mesh::Sphere::Sphere(float x, float y, float z, float r, uint mID) :
      m_c(Point(x, y, z)),
      m_r(r),
      m_mID(mID)
    {
      // no-op
    }

    Mesh::Triangle::Triangle(uint v0i, uint v1i, uint v2i, uint mID, uint oID) :
      m_mID(mID),
      m_oID(oID)
    {
      m_vID[0] = v0i;
      m_vID[1] = v1i;
      m_vID[2] = v2i;
    }

    Mesh::Triangle::Triangle()
    {
      // no-op
    }

    bool Mesh::Triangle::read(istream& in)
    {
      // Read vertex indices
      in.read(reinterpret_cast<char*>(&m_vID[0]), sizeof(uint));
      in.read(reinterpret_cast<char*>(&m_vID[1]), sizeof(uint));
      in.read(reinterpret_cast<char*>(&m_vID[2]), sizeof(uint));

      // Read material, object indices
      in.read(reinterpret_cast<char*>(&m_mID), sizeof(uint));
      in.read(reinterpret_cast<char*>(&m_oID), sizeof(uint));

      return in.good();
    }

    bool Mesh::Triangle::write(ostream& out) const
    {
      // Write vertex indices
      out.write(reinterpret_cast<const char*>(&m_vID[0]), sizeof(uint));
      out.write(reinterpret_cast<const char*>(&m_vID[1]), sizeof(uint));
      out.write(reinterpret_cast<const char*>(&m_vID[2]), sizeof(uint));

      // Write material, object indices
      out.write(reinterpret_cast<const char*>(&m_mID), sizeof(uint));
      out.write(reinterpret_cast<const char*>(&m_oID), sizeof(uint));

      return out.good();
    }

    Mesh::Vertex::Vertex(const Point&    p,
                         const Vector&   n,
                         const TexCoord& t) :
      m_p(p),
      m_n(n),
      m_t(t)
    {
      // no-op
    }

    Mesh::Vertex::Vertex()
    {
      // no-op
    }

    bool Mesh::Vertex::read(istream& in)
    {
      // Read point
      in.read(reinterpret_cast<char*>(&m_p[0]), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_p[1]), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_p[2]), sizeof(float));

      // Read normal
      in.read(reinterpret_cast<char*>(&m_n[0]), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_n[1]), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_n[2]), sizeof(float));

      // Read texture coordinates
      in.read(reinterpret_cast<char*>(&m_t[0]), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_t[1]), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_t[2]), sizeof(float));

      return in.good();
    }

    bool Mesh::Vertex::write(ostream& out) const
    {
      // Write point
      out.write(reinterpret_cast<const char*>(&m_p[0]), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_p[1]), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_p[2]), sizeof(float));

      // Write normal
      out.write(reinterpret_cast<const char*>(&m_n[0]), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_n[1]), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_n[2]), sizeof(float));

      // Write texture coordinates
      out.write(reinterpret_cast<const char*>(&m_t[0]), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_t[1]), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_t[2]), sizeof(float));

      return out.good();
    }


    ///////////////////////////////////////////////////////////////////////////
    // Constructors, destructor, & assignment operator

    Mesh::Mesh(const string& filename) :
      m_nbytes(0)
    {
      load(filename);
    }

    Mesh::Mesh(const Mesh& rhs) :
      m_nbytes(rhs.m_nbytes)
    {
      if (this == &rhs)
        return;

      // Copy data
      m_b = rhs.m_b;
      m_m = rhs.m_m;
      m_t = rhs.m_t;
      m_v = rhs.m_v;

      // Copy textures
      map<string, Texture*>::const_iterator iter = rhs.m_tmap.begin();
      while (iter != rhs.m_tmap.end())
      {
        m_tmap[iter->first] = (iter->second ? new Texture(*iter->second) : 0);
        ++iter;
      }
    }

    Mesh::Mesh() :
      m_nbytes(0)
    {
      // no-op
    }

    Mesh::~Mesh()
    {
      map<string, Texture*>::iterator iter = m_tmap.begin();
      while (iter != m_tmap.end())
        delete (iter++)->second;
    }

    Mesh& Mesh::operator=(const Mesh& rhs)
    {
      if (this == &rhs)
        return *this;

      // Copy data
      m_b = rhs.m_b;
      m_m = rhs.m_m;
      m_t = rhs.m_t;
      m_v = rhs.m_v;

      m_nbytes = rhs.m_nbytes;

      // Copy textures
      m_tmap.clear();
      map<string, Texture*>::const_iterator iter = rhs.m_tmap.begin();
      while (iter != rhs.m_tmap.end())
      {
        m_tmap[iter->first] = (iter->second ? new Texture(*iter->second) : 0);
        ++iter;
      }

      return *this;
    }


    ///////////////////////////////////////////////////////////////////////////
    // Geometry processing

    void Mesh::load(const string& filename)
    {
      /////////////////////////////////////////////////////////////////////////
      // Load geometry

      if (filename.rfind(".obj") != string::npos)
        loadOBJ(filename);
      else if (filename.rfind(".mini") != string::npos)
        loadML(filename);
      else if (filename.rfind(".iw") != string::npos)
        loadIW(filename);
      else
      {
        try
        {
          loadRF(filename);
        }
        catch (const Mesh::InvalidMesh& error)
        {
          // NOTE(cpg) - loadRF throws a null string if the file does not
          //             contain a valid Rayforce graphlog; in this case,
          //             throw a new exception with "Unrecognized ..."
          //             message

          // Convert to "Unrecognized ...", if necessary
          if (strcmp(error.what(), "") == 0)
          {
            string msg = "Unrecognized mesh format:  \"" + filename + "\"";
            throw Mesh::InvalidMesh(msg);
          }

          // Throw loadRF exception
          throw error;
        }
      }


      /////////////////////////////////////////////////////////////////////////
      // Validate geometry, materials

      if (m_v.size() == 0)
        throw Mesh::InvalidMesh("Invalid mesh:  no vertices found!");

      if (m_t.size() == 0)
        throw Mesh::InvalidMesh("Invalid mesh:  no triangles found!");

      if (m_m.size() == 0)
      {
        Material material("Mesh::Material::DefaultMaterial");

        material.m_rgb[Material::Ambient     ] = Mesh::DefaultColor[Material::Ambient     ];
        material.m_rgb[Material::Diffuse     ] = Mesh::DefaultColor[Material::Diffuse     ];
        material.m_rgb[Material::Specular    ] = Mesh::DefaultColor[Material::Specular    ];
        material.m_rgb[Material::Transmissive] = Mesh::DefaultColor[Material::Transmissive];
        material.m_rgb[Material::Emissive    ] = Mesh::DefaultColor[Material::Emissive    ];

        material.m_tex[Material::Ambient ]     = genTexture(material,
                                                            Material::Ambient);
        material.m_tex[Material::Diffuse ]     = genTexture(material,
                                                            Material::Diffuse);
        material.m_tex[Material::Specular]     = genTexture(material,
                                                            Material::Specular);
        material.m_tex[Material::Transmissive] = genTexture(material,
                                                            Material::Transmissive);
        material.m_tex[Material::Emissive]     = genTexture(material,
                                                            Material::Emissive);

        m_m.push_back(material);
      }


      /////////////////////////////////////////////////////////////////////////
      // Compute bounds

      vector<Vertex>::const_iterator iter = m_v.begin();
      while (iter != m_v.end())
      {
        m_b.extend(iter->m_p);
        ++iter;
      }
    }


    ///////////////////////////////////////////////////////////////////////////
    // Mesh persistence

    bool Mesh::read(istream& in)
    {
      uint mcnt, tcnt, vcnt;

      // Read metadata
      in.read(reinterpret_cast<char*>(&m_nbytes), sizeof(uint));
      in.read(reinterpret_cast<char*>(&mcnt),     sizeof(uint));
      in.read(reinterpret_cast<char*>(&tcnt),     sizeof(uint));
      in.read(reinterpret_cast<char*>(&vcnt),     sizeof(uint));

      // Read materials
      BufferedReader<Material, 1, true> mreader(in);
      mreader.init(true);

      m_m.clear();
      while (m_m.size() < mcnt)
      {
        Material material;
        mreader.read(material);
        m_m.push_back(material);
      }

      // Read triangles
      BufferedReader<Triangle, 1, true> treader(in);
      treader.init(true);

      m_t.clear();
      while (m_t.size() < tcnt)
      {
        Triangle triangle;
        treader.read(triangle);
        m_t.push_back(triangle);
      }

      // Read vertices
      BufferedReader<Vertex, 1, true> vreader(in);
      vreader.init(true);

      m_v.clear();
      while (m_v.size() < vcnt)
      {
        Vertex vertex;
        vreader.read(vertex);
        m_v.push_back(vertex);
      }

      return in.good();
    }

    bool Mesh::write(ostream& out) const
    {
      uint mcnt = m_m.size();
      uint tcnt = m_t.size();
      uint vcnt = m_v.size();

      // Write metadata
      out.write(reinterpret_cast<const char*>(&m_nbytes), sizeof(uint));
      out.write(reinterpret_cast<const char*>(&mcnt),     sizeof(uint));
      out.write(reinterpret_cast<const char*>(&tcnt),     sizeof(uint));
      out.write(reinterpret_cast<const char*>(&vcnt),     sizeof(uint));

      // Write materials
      BufferedWriter<Material, 10, true> mwriter(out);
      mwriter.init(true);

      vector<Material>::const_iterator miter = m_m.begin();
      while (miter != m_m.end())
        mwriter.write(*miter++);
      mwriter.flush();

      // Write triangles
      BufferedWriter<Triangle, 10, true> twriter(out);
      twriter.init(true);

      vector<Triangle>::const_iterator titer = m_t.begin();
      while (titer != m_t.end())
        twriter.write(*titer++);
      twriter.flush();

      // Write vertices
      BufferedWriter<Vertex, 10, true> vwriter(out);
      vwriter.init(true);

      vector<Vertex>::const_iterator viter = m_v.begin();
      while (viter != m_v.end())
        vwriter.write(*viter++);
      vwriter.flush();

      return out.good();
    }


    ///////////////////////////////////////////////////////////////////////////
    // Load MiniLight geometry file

    void Mesh::loadML(const string& filename)
    {
      ifstream fin(filename.c_str());
      if (!fin.is_open())
      {
        string msg = "Failed to open \"" + filename + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }

      // Parse scene file
      string junk;
      fin >> junk;

      // Parse image settings
      uint niters, xres, yres;
      fin >> niters >> xres >> yres;

      // Parse camera parameters
      Vector eye, dir;
      float  hfov;
      fin >> eye >> dir >> hfov;

      // Parse scene parameters
      core::Float::RGB skyEmission, grndEmission;
      fin >> skyEmission >> grndEmission;

      loadML(fin);

      fin.close();


      /////////////////////////////////////////////////////////////////////////
      // Update in-memory size

      m_nbytes  = m_t.size()*sizeof(Triangle);
      m_nbytes += m_m.size()*sizeof(Material);
      m_nbytes += m_v.size()*sizeof(Vertex);
    }

    void Mesh::loadML(ifstream& fin)
    {
      //! @todo TODO(cpg) - implement duplicate vertex filtering

      // Read vertex data
      Vertex v0, v1, v2;
      while (fin >> v0.m_p >> v1.m_p >> v2.m_p)
      {
        // Read material data
        core::Float::RGB diffuse, emissive;
        fin >> diffuse >> emissive;

        // Construct edges, face normal
        const Vector e0 = v1.m_p - v0.m_p;
        const Vector e1 = v2.m_p - v0.m_p;
        //      Vector n  = Cross(e0, e1).normal();
        const Vector n  = Cross(e0, e1).normal();

        // NOTE(jsh) - this causes errors for scenes with small triangles
        //             because simply continuing will result in a null pointer
        /*
        // Skip degenerate triangles
        if (n.length() <= Epsilon)
          continue;

        n.normalize();
        */

        // Finish construction
        v0.m_n = n;
        v1.m_n = n;
        v2.m_n = n;

        diffuse.clamp(0.f, 1.f);
        emissive.clamp(0.f, numeric_limits<float>::max());

        // Create material
        Material material;

        material.m_rgb[Material::Diffuse ] = diffuse;
        material.m_rgb[Material::Emissive] = emissive;

        material.m_tex[Material::Ambient ]     = genTexture(material,
                                                            Material::Ambient );
        material.m_tex[Material::Diffuse ]     = genTexture(material,
                                                            Material::Diffuse );
        material.m_tex[Material::Specular]     = genTexture(material,
                                                            Material::Specular);
        material.m_tex[Material::Transmissive] = genTexture(material,
                                                            Material::Transmissive);
        material.m_tex[Material::Emissive]     = genTexture(material,
                                                            Material::Emissive);

        // Add vertex, material data
        m_v.push_back(v0);
        m_v.push_back(v1);
        m_v.push_back(v2);

        m_m.push_back(material);

        // Construct face
        const uint nverts = m_v.size();
        const uint nmats  = m_m.size();

        // Add face
        m_t.push_back(Triangle(nverts-3, nverts-2, nverts-1, nmats-1));
      }
    }


    ///////////////////////////////////////////////////////////////////////////
    // Helper class

    typedef pair<int, int>      intPair2;
    typedef pair<int, intPair2> intPair3;
    typedef pair<int, intPair3> intPair4;

    /*! @struct Tuple4
        @brief 4-tuple of integers for use by Mesh.
    */
    struct Tuple4
    {
      Tuple4(int, int, int, int);

      intPair4 m_data;
    };

    Tuple4::Tuple4(int a, int b, int c, int d) :
      m_data(a, intPair3(b, intPair2(c, d)))
    {
      // no-op
    }

    bool operator<(const Tuple4& t1, const Tuple4& t2)
    {
      return (t1.m_data < t2.m_data);
    }

    ostream& operator<<(ostream& out, const Tuple4& t4)
    {
      int one   = t4.m_data.first;
      int two   = t4.m_data.second.first;
      int three = t4.m_data.second.second.first;
      int four  = t4.m_data.second.second.second;

      out << '(' << one << ", " << two << ", " << three << ", " << four << ')';

      return out;
    }



    ///////////////////////////////////////////////////////////////////////////
    // Load OBJ geometry file

    void Mesh::loadOBJ(const string& filename)
    {
      // Open OBJ file
#ifdef USE_IFSTREAM
      ifstream fin(filename.c_str());
      if (!fin.is_open())
      {
        string msg = "Failed to open \"" + filename + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }
#else
      /*! @todo TODO(cpg) - cplusplus-ify me! */
      FILE* fp = fopen(filename.c_str(), "rt");
      if (!fp)
      {
        string msg = "Failed to open \"" + filename + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }
#endif // USE_IFSTREAM

      char line[1024];
      char token[1024];

      // Per-vertex data
      vector<Point>    in_p;
      vector<TexCoord> in_t;
      vector<Vector>   in_n;

      vector<int>      clean_p;
      vector<int>      clean_t;
      vector<int>      clean_n;
      map<Tuple4, int> clean_map;

      // Per-face data
      vector<int> vertex0, vertex1, vertex2;
      vector<int> mID;
      vector<int> oID;

      map<string, int> mID_map;
      vector<Material> clean_m;
      map<string, int> oID_map;

      // Spheres
      vector<Sphere> spheres;

      // Parse OBJ file
      uint currentMaterial = 0;
      uint currentGroup    = 0;
      uint currentObject   = 0;

      while (true)
      {
        float x, y, z, r;

#ifdef USE_IFSTREAM
        if (!fin.getline(line, 1024))
          break;
#else
        if (!fgets(line, 1024, fp))
          break;
#endif // USE_IFSTREAM

        // NOTE(cpg) - Leading whitespace on all sscanf format strings tells
        //             the function to discard all leading whitespace on the
        //             input strings
        if (sscanf(line, " v %f %f %f", &x, &y, &z) == 3)
        {
          in_p.push_back(Point(x, y, z));
        }
        else if (sscanf(line, " vt %f %f", &x, &y) == 2)
        {
          in_t.push_back(TexCoord(x, y, 0.f));
        }
        else if (sscanf(line, " vn %f %f %f", &x, &y, &z) == 3)
        {
          in_n.push_back(Vector(x, y, z));
        }
        else if (sscanf(line, " mtllib %s", token) == 1)
        {
          string tokenS(token);
          string filenameS(filename);

          // NOTE(cpg) - strlen("mtllib ") == 7
          string remainder(line + 7 + tokenS.length());

          // Trim trailing whitespace characters
          const char*  delim = " \t\n\v\f\r";
          const size_t end = remainder.find_last_not_of(delim);

          tokenS += remainder.substr(0, end+1);

          loadMTL(clean_m, mID_map, tokenS, filenameS);
        }
        else if (sscanf(line, " usemtl %s", token) == 1)
        {
          currentMaterial = mID_map[token];
        }
        else if (sscanf(line, " s %d", &currentGroup) == 1)
        {
          // no-op: currentGroup set in sscanf
        }
        else if (sscanf(line, " sphere %f %f %f %f", &x, &y, &z, &r) == 4)
        {
          Sphere sphere(x, y, z, r, currentMaterial);
          spheres.push_back(sphere);
        }
        else if(sscanf(line, " o %s", token) == 1)
        {
          // Is this object already in the map?
          map<string, int>::iterator iter = oID_map.find(token);
          if (iter == oID_map.end())
          {
            currentObject  = oID_map.size();
            oID_map[token] = currentObject;
          }
          else
          {
            currentObject = (*iter).second;
          }
        }
        else
        {
          int vp0 = Vertex::Undefined + 1;
          int vp1 = Vertex::Undefined + 1;
          int vp2 = Vertex::Undefined + 1;
          int vp3 = Vertex::Undefined + 1;

          int vt0 = Vertex::Undefined + 1;
          int vt1 = Vertex::Undefined + 1;
          int vt2 = Vertex::Undefined + 1;
          int vt3 = Vertex::Undefined + 1;

          int vn0 = Vertex::Undefined + 1;
          int vn1 = Vertex::Undefined + 1;
          int vn2 = Vertex::Undefined + 1;
          int vn3 = Vertex::Undefined + 1;

          if (sscanf(line, " f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d",
                     &vp0, &vt0, &vn0,
                     &vp1, &vt1, &vn1,
                     &vp2, &vt2, &vn2,
                     &vp3, &vt3, &vn3) >= 9 ||
              sscanf(line, " f %d//%d %d//%d %d//%d %d//%d",
                     &vp0, &vn0,
                     &vp1, &vn1,
                     &vp2, &vn2,
                     &vp3, &vn3) >= 6 ||
              sscanf(line, " f %d/%d %d/%d %d/%d %d/%d",
                     &vp0, &vt0,
                     &vp1, &vt1,
                     &vp2, &vt2,
                     &vp3, &vt3) >= 6 ||
              sscanf(line, " f %d %d %d %d",
                     &vp0,
                     &vp1,
                     &vp2,
                     &vp3) >= 3)
          {
            const int poffset = in_p.size();
            const int toffset = in_t.size();
            const int noffset = in_n.size();

            vp0 += (vp0 < 0 ? poffset : -1);
            vp1 += (vp1 < 0 ? poffset : -1);
            vp2 += (vp2 < 0 ? poffset : -1);
            vp3 += (vp3 < 0 ? poffset : -1);

            vt0 += (vt0 < 0 ? toffset : -1);
            vt1 += (vt1 < 0 ? toffset : -1);
            vt2 += (vt2 < 0 ? toffset : -1);
            vt3 += (vt3 < 0 ? toffset : -1);

            vn0 += (vn0 < 0 ? noffset : -1);
            vn1 += (vn1 < 0 ? noffset : -1);
            vn2 += (vn2 < 0 ? noffset : -1);
            vn3 += (vn3 < 0 ? noffset : -1);

            // Add new vertices (as necessary)
#if 1
            // NOTE(cpg) - the 'else' block of this code generates a unique
            //             smoothing group per face when the OBJ file does
            //             not have any smoothing group tags ('s <int>')
            //
            //             that, in turn, leads to (unnecessary) vertex
            //             duplication because of the way we classify unique
            //             vertices, so disable smoothing-group-per-face for
            //             now
            int smoothingGroup = currentGroup;
#else
            int smoothingGroup = (currentGroup > 0 ?
                                  currentGroup :
                                  -1 - (int)vertex0.size());
#endif

            Tuple4 id0(vp0, vt0, vn0, smoothingGroup);
            if (clean_map.find(id0) == clean_map.end())
            {
              clean_map[id0] = clean_p.size();
              clean_p.push_back(vp0);
              clean_t.push_back(vt0);
              clean_n.push_back(vn0);
            }

            Tuple4 id1(vp1, vt1, vn1, smoothingGroup);
            if (clean_map.find(id1) == clean_map.end())
            {
              clean_map[id1] = clean_p.size();
              clean_p.push_back(vp1);
              clean_t.push_back(vt1);
              clean_n.push_back(vn1);
            }

            Tuple4 id2(vp2, vt2, vn2, smoothingGroup);
            if (clean_map.find(id2) == clean_map.end())
            {
              clean_map[id2] = clean_p.size();
              clean_p.push_back(vp2);
              clean_t.push_back(vt2);
              clean_n.push_back(vn2);
            }

            vertex0.push_back(clean_map[id0]);
            vertex1.push_back(clean_map[id1]);
            vertex2.push_back(clean_map[id2]);

            mID.push_back(currentMaterial);
            oID.push_back(currentObject);

            if (vp3 != Vertex::Undefined)
            {
              Tuple4 id3(vp3, vt3, vn3, smoothingGroup);
              if (clean_map.find(id3) == clean_map.end())
              {
                clean_map[id3] = clean_p.size();
                clean_p.push_back(vp3);
                clean_t.push_back(vt3);
                clean_n.push_back(vn3);
              }

              vertex0.push_back(clean_map[id0]);
              vertex1.push_back(clean_map[id2]);
              vertex2.push_back(clean_map[id3]);

              mID.push_back(currentMaterial);
              oID.push_back(currentObject);
            }
          }
        }
      }

#ifdef USE_IFSTREAM
      fin.close();
#else
      fclose(fp);
#endif // USE_IFSTREAM

#if 1
      // XXX(cpg) - fixme!
      ato::core::io::cerr << "ato::core::Mesh::loadOBJ : Warning - texture coordinate adjustments disabled" << std::endl;
#else
      // Make sure all texture coordinates are positive
      const uint ntex = in_t.size();
      if (ntex > 0)
      {
        TexCoord tmin = in_t[0];
        for (uint i = 1; i < ntex; ++i)
        {
          tmin[0] = (tmin[0] < in_t[i][0] ? tmin[0] : in_t[i][0]);
          tmin[1] = (tmin[1] < in_t[i][1] ? tmin[1] : in_t[i][1]);
          tmin[2] = 0.f;
        }

        TexCoord correction;
        correction[0] = (tmin[0] < 0.f ? -floorf(tmin[0]) : 0.f);
        correction[1] = (tmin[1] < 0.f ? -floorf(tmin[1]) : 0.f);
        correction[2] = 0.f;

        for (uint i = 0; i < ntex; ++i)
          in_t[i] += correction;
      }
#endif

#ifndef ATO_CORE_IGNORE_SPHERE_PRIMITIVES
      // Add spheres
      for (uint i = 0; i < spheres.size(); ++i)
      {
        const float& r = spheres[i].m_r;

        in_p.push_back(spheres[i].m_c);
        in_p.push_back(Point(r, r, r));

        clean_p.push_back(in_p.size() - 2);
        clean_p.push_back(in_p.size() - 1);

        vertex0.push_back(Vertex::Undefined);
        vertex1.push_back(clean_p.size() - 2);
        vertex2.push_back(clean_p.size() - 1);

        mID.push_back(spheres[i].m_mID);

        clean_t.push_back(Vertex::Undefined);
        clean_t.push_back(Vertex::Undefined);

        clean_n.push_back(Vertex::Undefined);
        clean_n.push_back(Vertex::Undefined);
      }
#endif

      // Populate mesh
      const uint nverts = clean_p.size();
      const uint ntris  = vertex0.size();
      const uint nmats  = clean_m.size();

      // Geometry data
      const float vtotal = float(nverts*sizeof(Vertex));
      const float ftotal = float(ntris*sizeof(Triangle));
      const float gtotal = vtotal + ftotal;

      /*
      Output("Geometry data:" << gtotal/1024.f << endl);
      Output("  # vertices  = " << nverts << ", "
             << vtotal/1024.f << " KB" << endl);
      Output("  # faces     = " << ntris << ", "
             << ftotal/1024.f << " KB" << endl);
      */

      // Texture data
      const float ttotal = float(nmats*sizeof(Material));

      /*
      Output("Texture data:  " ttotal/1024.f << endl);
      Output("  # materials = " << nmats << endl);
      */

      m_nbytes += size_t(gtotal + ttotal);

      // Fill in per-vertex data
      m_v.resize(nverts);
      for (uint i = 0; i < nverts; ++i)
      {
        const uint& pidx = clean_p[i];
        const uint& tidx = clean_t[i];
        const uint& nidx = clean_n[i];

        const Point& vp = in_p[pidx];

        TexCoord vt;
        vt[0] = (tidx == Vertex::Undefined ? 0.f : in_t[tidx][0]);
        vt[1] = (tidx == Vertex::Undefined ? 0.f : in_t[tidx][1]);
        vt[2] = 0.f;

        Vector vn;
        vn[0] = (nidx == Vertex::Undefined ? 0.f : in_n[nidx][0]);
        vn[1] = (nidx == Vertex::Undefined ? 0.f : in_n[nidx][1]);
        vn[2] = (nidx == Vertex::Undefined ? 0.f : in_n[nidx][2]);

        m_v[i] = Vertex(vp, vn.normal(), vt);
      }

      // Fill in face data
      m_t.resize(ntris);
      for (uint i = 0; i < ntris; ++i)
      {
        // Interpolate normals
        const uint& v0i = vertex0[i];
        const uint& v1i = vertex1[i];
        const uint& v2i = vertex2[i];

        if (v0i == Vertex::Undefined)
        {
          m_t[i] = Triangle(v0i, v1i, v2i, mID[i], oID[i]);
          continue;
        }

        const Point& v0 = m_v[v0i].m_p;
        const Vector e0 = m_v[v1i].m_p - v0;
        const Vector e1 = m_v[v2i].m_p - v0;
              Vector n  = Cross(e0, e1);

        // NOTE(jsh) - this causes errors for scenes with small triangles
        /*
        // Skip degenerate triangles
        if (n.length() <= Epsilon)
          continue;
        */

        if (clean_n[v0i] == Vertex::Undefined)
          m_v[v0i].m_n += n;

        if (clean_n[v1i] == Vertex::Undefined)
          m_v[v1i].m_n += n;

        if (clean_n[v2i] == Vertex::Undefined)
          m_v[v2i].m_n += n;

        // Add face
        m_t[i] = Triangle(v0i, v1i, v2i, mID[i], oID[i]);

        /*
        Output("face[" << i << "]:" << endl);
        Output("  v0 = " << v[v0i] << endl);
        Output("  v1 = " << v[v1i] << endl);
        Output("  v2 = " << v[v2i] << endl);
        */
      }

      // Renormalize
      for (uint i = 0; i < nverts; ++i)
        m_v[i].m_n.normalize();

      // Fill in material data
      m_m.resize(nmats);
      for (uint i = 0; i < nmats; ++i)
      {
        Material& material = clean_m[i];

        material.m_tex[Material::Ambient ]     = genTexture(material,
                                                            Material::Ambient );
        material.m_tex[Material::Diffuse ]     = genTexture(material,
                                                            Material::Diffuse );
        material.m_tex[Material::Specular]     = genTexture(material,
                                                            Material::Specular);
        material.m_tex[Material::Transmissive] = genTexture(material,
                                                            Material::Transmissive);
        material.m_tex[Material::Emissive]     = genTexture(material,
                                                            Material::Emissive);

        m_m[i] = material;
      }
    }


    ///////////////////////////////////////////////////////////////////////////
    // Load OBJ material library

    void Mesh::loadMTL(      vector<Material>& clean_m,
                             map<string, int>& mID_map,
                             string&           fnameMTL,
                       const string&           filename)
    {
      char line[1024]  = {};
      char token[1024] = {};

      float x, y, z;

      // Open MTL library
      matchDirectory(fnameMTL, filename);
#ifdef USE_IFSTREAM
      ifstream fin(fnameMTL.c_str());
      if (!fin.is_open())
      {
        string msg = "Failed to open \"" + fnameMTL + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }

      // Parse MTL library
      while (true)
      {
        if (!fin.getline(line, 1024))
          break;

#else
      /*! @todo TODO(cpg) - cplusplus-ify me! */
      FILE* fp = fopen(fnameMTL.c_str(), "rt");
      if (!fp)
      {
        string msg = "Failed to open \"" + fnameMTL + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }

      // Parse MTL library
      while (true)
      {
        if (!fgets(line, 1024, fp))
          break;
#endif

        if (sscanf(line, " newmtl %s", token) == 1)
        {
          // Create material
          Material material(token);
          clean_m.push_back(material);
          mID_map[token] = clean_m.size()-1;
        }
        else if (sscanf(line, " Ka %f %f %f", &x, &y, &z) == 3)
        {
          core::Float::RGB& ambient = clean_m.back().m_rgb[Material::Ambient];
          ambient = core::Float::RGB(x, y, z);
          ambient.clamp(0.f, 1.f);
        }
        else if (sscanf(line, " Kd %f %f %f", &x, &y, &z) == 3)
        {
          core::Float::RGB& diffuse = clean_m.back().m_rgb[Material::Diffuse];
          diffuse = core::Float::RGB(x, y, z);
          diffuse.clamp(0.f, 1.f);
        }
        else if (sscanf(line, " Ks %f %f %f", &x, &y, &z) == 3)
        {
          core::Float::RGB& specular = clean_m.back().m_rgb[Material::Specular];
          specular = core::Float::RGB(x, y, z);
          specular.clamp(0.f, 1.f);
        }
        else if (sscanf(line, " Tf %f %f %f", &x, &y, &z) == 3)
        {
          core::Float::RGB& trans = clean_m.back().m_rgb[Material::Transmissive];
          trans = core::Float::RGB(x, y, z);
          trans.clamp(0.f, 1.f);
        }
        else if (sscanf(line, " Ke %f %f %f", &x, &y, &z) == 3)
        {
          core::Float::RGB& emissive = clean_m.back().m_rgb[Material::Emissive];
          emissive = core::Float::RGB(x, y, z);
          emissive.clamp(0.f, numeric_limits<float>::max());
        }
        else if (sscanf(line, " map_Kd %s", token) == 1)
        {
          // Two forms of map_Kd
          float uscale = 1.f;
          float vscale = 1.f;
          if (sscanf(line, "map_Kd -s %f %f %s",
                     &uscale, &vscale, token) == 3)
          {
            // no-op: uscale, vscale, token set in sscanf
          }
          else if (sscanf(line, "map_Kd %s", token) == 1)
          {
            // no-op:  token set in sscanf
          }

          string tstring(token);
          matchDirectory(tstring, filename);

          Material& m(clean_m.back());
          m.m_filename[Material::Diffuse] = tstring;
          m.m_uscale[Material::Diffuse]   = (uscale < 1.f ? 1.f/uscale : uscale);
          m.m_vscale[Material::Diffuse]   = (vscale < 1.f ? 1.f/vscale : vscale);
        }
        else if (sscanf(line, " map_Ks %s", token) == 1)
        {
          string tstring(token);
          matchDirectory(tstring, filename);
          clean_m.back().m_filename[Material::Specular] = tstring;
        }
        else if (sscanf(line, " map_Ke %s", token) == 1)
        {
          string tstring(token);
          matchDirectory(tstring, filename);
          clean_m.back().m_filename[Material::Emissive] = tstring;
        }
        else if (sscanf(line, " Ns %f", &x) == 1)
        {
          clean_m.back().m_ns = (x > 0.f ? x : 0.f);
        }
        else if (sscanf(line, " R0 %f", &x) == 1 ||
                 sscanf(line, " Ni %f", &x) == 1)
        {
          clean_m.back().m_ni = (x > 0.f ? x : 0.f);
        }
        else if (sscanf(line, " Tr %f", &x) == 1 ||
                 sscanf(line, " d %f",  &x) == 1)
        {
          clean_m.back().m_tr = x;
        }
        /*
        else
        {
          string msg = "Unrecognized token \"" + line + "\" in OBJ file";
          throw Mesh::InvalidMesh(msg);
        }
        */
      }

#ifdef USE_IFSTREAM
      fin.close();
#else
      fclose(fp);
#endif
    }


    ///////////////////////////////////////////////////////////////////////////
    // Load IW geometry file

    void Mesh::loadIW(const string& filename)
    {
      // Open IW file
      /*! @todo TODO(cpg) - cplusplus-ify me! */
      FILE* fp = fopen(filename.c_str(), "rt");
      if (!fp)
      {
        string msg = "Failed to open \"" + filename + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }

      char line[1024];
      char token[1024];
      char type[10];

      uint  nverts   = 0;
      uint  nnormals = 0;
      uint  ntris    = 0;
      uint  nmats    = 0;
      char* rc;

      // Parse IW file
      m_nbytes = 0;
      while (true)
      {
        if (!fgets(line, 1024, fp))
          break;

        // NOTE(cpg) - Leading whitespace on all sscanf format strings tells
        //             the function to discard all leading whitespace on the
        //             input strings
        if (sscanf(line, " vertices: %d", &nverts) == 1)
        {
          if (nnormals > 0 && nverts != nnormals)
          {
            string msg = "Number of vertices (" + to_string(nverts) +
              ") does not match number of normals (" + to_string(nnormals) +
              ")";
            throw Mesh::InvalidMesh(msg);
          }

          if (m_v.size() != nverts)
          {
            m_v.resize(nverts);
            m_nbytes += nverts*sizeof(Vertex);
          }

          float x, y, z;
          for (uint i = 0; i < nverts; ++i)
          {
            rc = fgets(line, 1024, fp);
            sscanf(line, "%f %f %f", &x, &y, &z);

            // Add vertex position
            m_v[i].m_p = Point(x, y, z);
          }
        }
        else if (sscanf(line, " vtxnormals: %d", &nnormals) == 1)
        {
          if (nverts > 0 && nnormals != nverts)
          {
            string msg = "Number of normals (" + to_string(nnormals) +
              ") does not match number of vertices (" + to_string(nverts) +
              ")";
            throw Mesh::InvalidMesh(msg);
          }

          if (m_v.size() != nverts)
          {
            m_v.resize(nverts);
            m_nbytes += nnormals*sizeof(Vertex);
          }

          float x, y, z;
          for (uint i = 0; i < nverts; ++i)
          {
            rc = fgets(line, 1024, fp);
            sscanf(line, "%f %f %f", &x, &y, &z);

            // Add vertex normal
            m_v[i].m_n = Vector(x, y, z).normal();
          }
        }
        else if (sscanf(line, " triangles: %d", &ntris) == 1)
        {
          m_t.resize(ntris);
          m_nbytes += ntris*sizeof(Triangle);

          uint v0i, v1i, v2i, mID;
          for (uint i = 0; i < ntris; ++i)
          {
            rc = fgets(line, 1024, fp);
            sscanf(line, "%d %d %d %d", &v0i, &v1i, &v2i, &mID);

            // Add face
            m_t[i] = Triangle(v0i, v1i, v2i, mID);
          }
        }
        else if (sscanf(line, " shaders %d", &nmats) == 1)
        {
          map<string, int> mID_map;
          vector<Material> clean_m;
          vector<int>      actual_mid(nmats);

          uint  mID;
          float r, g, b, exp, r0;
          uint  i = 0;
          while (i < nmats)
          {
            rc = fgets(line, 1024, fp);

            // Build material string
            if (sscanf(line, " # %s", token))
              continue;
            else if (sscanf(line, " shader %d %s (%f,%f,%f)",
                            &mID, type, &r, &g, &b) == 5)
              sprintf(token, "%s (%f,%f,%f)", type, r, g, b);
            else if (sscanf(line, " shader %d %s (%f,%f,%f) %f %f",
                            &mID, type, &r, &g, &b, &exp, &r0) == 6)
              sprintf(token, "%s (%f,%f,%f) %f %f", type, r, g, b, exp, r0);

            if (mID > nmats)
            {
              string msg = "Invalid material id (" + to_string(mID) + ")";
              throw Mesh::InvalidMesh(msg);
            }

            // Search for material string, adding new materials as necessary
            if (mID_map.find(token) == mID_map.end())
            {
              // Add a new material
              Material material(token);
              if (strcmp(type, "diffuse") == 0)
              {
                core::Float::RGB& diffuse = material.m_rgb[Material::Diffuse];
                diffuse = core::Float::RGB(r, g, b);
                diffuse.clamp(0.f, 1.f);
              }
              else if (strcmp(type, "emissive") == 0)
              {
                core::Float::RGB& emissive = material.m_rgb[Material::Emissive];
                emissive = core::Float::RGB(r, g, b);
                emissive.clamp(0.f, numeric_limits<float>::max());
              }
              else if (strcmp(type, "coupled") == 0)
              {
                //! @bug XXX(cpg) - not yet (fully) implemented, but why not!?!
                core::Float::RGB& specular = material.m_rgb[Material::Specular];
                specular = core::Float::RGB(r, g, b);
                specular.clamp(0.f, 1.f);
              }
              else
              {
                string msg = "Invalid shading model (" + to_string(type) + ")";
                throw Mesh::InvalidMesh(msg);
              }

              clean_m.push_back(material);
              mID_map[token] = clean_m.size()-1;
            }

            actual_mid[mID] = mID_map[token];
            ++i;
          }

          nmats   = clean_m.size();
          m_nbytes += nmats*sizeof(Material);

          // Fill in material data
          for (uint i = 0; i < nmats; ++i)
          {
            Material& material = clean_m[i];

            material.m_tex[Material::Ambient ]     = genTexture(material,
                                                                Material::Ambient );
            material.m_tex[Material::Diffuse ]     = genTexture(material,
                                                                Material::Diffuse );
            material.m_tex[Material::Specular]     = genTexture(material,
                                                                Material::Specular);
            material.m_tex[Material::Transmissive] = genTexture(material,
                                                                Material::Transmissive);
            material.m_tex[Material::Emissive]     = genTexture(material,
                                                                Material::Emissive);

            m_m.push_back(material);
          }

          // Update material ids
          for (uint i = 0; i < ntris; ++i)
            m_t[i].m_mID = actual_mid[m_t[i].m_mID];
        }
      }

      fclose(fp);
    }

    void Mesh::loadRF(const string& filename)
    {
      ifstream fin(filename.c_str());
      if (!fin.is_open())
      {
        string msg = "Failed to open \"" + filename + "\" for reading";
        throw Mesh::InvalidMesh(msg);
      }


      /////////////////////////////////////////////////////////////////////////
      // Read Rayforce graph log - triangles only!
      //
      //   Format:
      //     Count TriangleCount SectorCount NodeCount
      //     Triangle TriangeIndex ; v0.x v0.y v0.z ; v1.x v1.y v1.z ;
      //       v2.x v2.y v2.z
      //     Sector SectorIndex ; EdgeMinX EdgeMaxX EdgeMinY EdgeMaxY EdgeMinZ
      //       EdgeMaxZ ; LinkFlags ; LinkMinX LinkMaxX LinkMinY LinkMaxY
      //       LinkMinZ LinkMaxZ ; TriCount ; [TriIndex0] [TriIndex1] ...
      //     Node NodeIndex ; Axis Plane ; LinkFlags ; LinkMin LinkMax
      //
      //   Also, note that the following code removes duplicate vertices and
      //   adjusts triangle faces accordingly

      map<string, uint> clean_v;

      m_v.clear();
      m_m.clear();
      m_t.clear();
      m_tmap.clear();


      /////////////////////////////////////////////////////////////////////////
      // Read header
      //   Count %d %d %d

      string junk;
      uint   ntris, nsectors, nnodes;
      fin >> junk >> ntris >> nsectors >> nnodes;
      if (junk != "Count" || !fin.good())
      {
        // NOTE(cpg) - assume the file isn't a valid Rayforce graphlog, so
        //             throw an exception with null message, which is then
        //             converted to "Unrecognized ..." exception in
        //             Mesh::load(...) above

        throw Mesh::InvalidMesh("");
      }

      // Allocate memory
      m_t.resize(ntris);
      m_v.resize(3*ntris);


      /////////////////////////////////////////////////////////////////////////
      // Read triangles
      //   Triangle %d ; %f %f %f ; %f %f %f ; %f %f %f

      // Timer  timer;
      uint   nt = 0;
      uint   nv = 0;

      uint   tidx;
      Point  p[3];
      char   v_cstr[1024];
      string vidx;
      uint   vID[3];
      for (uint i = 0; i < ntris; ++i)
      {
        fin >> junk;
        if (junk != "Triangle")
        {
          string msg = "Expecting \"Triangle\", found \"" + junk + "\" instead";
          throw Mesh::InvalidMesh(msg);
        }

        // NOTE(cpg) - no per-vertex normal, texcoord data!
        fin >> tidx
            >> junk >> p[0][0] >> p[0][1] >> p[0][2]
            >> junk >> p[1][0] >> p[1][1] >> p[1][2]
            >> junk >> p[2][0] >> p[2][1] >> p[2][2];

        // Compute face normal
        const Vector e0(p[2] - p[0]);
        const Vector e1(p[1] - p[0]);
        const Vector n = Cross(e0, e1).normal();

        // New vertices?  Depends on location and face to which it belongs
        //   (determined using current face normal)
        for (uint j = 0; j < 3; ++j)
        {
          sprintf(v_cstr, "%f %f %f %f %f %f",
                  p[j][0], p[j][1], p[j][2],
                  n[0],    n[1],    n[2]);
          vidx = string(v_cstr);

          map<string, uint>::const_iterator iter = clean_v.find(vidx);
          if (iter == clean_v.end())
          {
            // Add new vertex
            m_v[nv]       = Vertex(p[j], n, TexCoord());
            vID[j]        = nv++;
            clean_v[vidx] = vID[j];

            continue;
          }

          // Duplicate vertex, save index
          vID[j] = iter->second;
        }

        // Add triangle
        m_t[nt++] = Triangle(vID[0], vID[1], vID[2], 0 /* mID */);
      }

      m_v.resize(nv);
      m_t.resize(nt);

      /*
      cout << "Took " << timer << " seconds to load/clean " << ntris
           << " triangles" << endl;
      */

      fin.close();


      /////////////////////////////////////////////////////////////////////////
      // Update in-memory size

      m_nbytes  = m_t.size()*sizeof(Triangle);
      m_nbytes += m_m.size()*sizeof(Material);
      m_nbytes += m_v.size()*sizeof(Vertex);
    }


    ///////////////////////////////////////////////////////////////////////////
    // Generate texture

    Texture* Mesh::genTexture(const Material&      material,
                              const Material::Type type)
    {
      const string& fname(material.m_filename[type]);
            string  idx;
      if (fname == "")
      {
        const core::Float::RGB& rgb(material.m_rgb[type]);
        char idx_cstr[1024];

        sprintf(idx_cstr, "rgb %f %f %f", rgb[0], rgb[1], rgb[2]);
        idx = string(idx_cstr);

        if (m_tmap[idx] == 0)
        {
          m_tmap[idx] = new Texture(rgb,
                                    material.m_uscale[type],
                                    material.m_vscale[type]);
        }
      }
      else
      {
        idx = fname;

        if (m_tmap[idx] == 0)
        {
          m_tmap[idx] = new Texture(fname,
                                    material.m_uscale[type],
                                    material.m_vscale[type]);
        }
      }

      return m_tmap[idx];
    }


    ///////////////////////////////////////////////////////////////////////////
    // Stream I/O operators

    istream& operator>>(istream& in, Mesh::Material& m)
    {
      in >> m.m_name;
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        in >> m.m_rgb[i];
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        in >> m.m_filename[i];
      int ijunk;
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        in >> ijunk;
      char cjunk;
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        in >> cjunk >> m.m_uscale[i] >> m.m_vscale[i] >> cjunk;
      in >> m.m_ns >> m.m_ni >> m.m_tr;

      return in;
    }

    ostream& operator<<(ostream& out, const Mesh::Material& m)
    {
      out << m.m_name << ' ';
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        out << m.m_rgb[i] << ' ';
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        out << m.m_filename[i] << ' ';
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        out << std::hex << m.m_tex[i] << ' ';
      for (uint i = 0; i < Mesh::Material::ntypes; ++i)
        out << '(' << m.m_uscale[i] << ' ' << m.m_vscale[i] << ") ";
      out << m.m_ns << ' ' << m.m_ni << ' ' << m.m_tr;

      return out;
    }

    istream& operator>>(istream& in, Mesh::Triangle& t)
    {
      in >> t.m_vID[0] >> t.m_vID[1] >> t.m_vID[2] >> t.m_mID >> t.m_oID;
      return in;
    }

    ostream& operator<<(ostream& out, const Mesh::Triangle& t)
    {
      out << t.m_vID[0] << ' ' << t.m_vID[1] << ' ' << t.m_vID[2] << ' '
          << t.m_mID << ' ' << t.m_oID;

      return out;
    }

    istream& operator>>(istream& in, Mesh::Vertex& v)
    {
      in >> v.m_p >> v.m_n >> v.m_t;
      return in;
    }

    ostream& operator<<(ostream& out, const Mesh::Vertex& v)
    {
      out << v.m_p << ' ' << v.m_n << ' ' << v.m_t;
      return out;
    }

  } // namespace core

} // namespace ato
