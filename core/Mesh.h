
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2013, Ethan Kerzner <ethan.kerzner[]gmail org>
 * Copyright (c) 2012, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef core_Mesh_h
#define core_Mesh_h

#include <iosfwd>
#include <map>
#include <string>
#include <vector>

#include <core/ato_coreExports.h>
#include <core/BBox.h>
#include <core/Exception.h>
#include <core/RGB.t>
#include <core/TexCoord.h>

#include <math/Math_Float.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::exception;
    using std::ifstream;
    using std::map;
    using std::ostream;
    using std::string;
    using std::vector;

    using core::Float::TexCoord;

    using math::Float::Point;
    using math::Float::Vector;


    ///////////////////////////////////////////////////////////////////////////
    // Forward declarations

    class Texture;


    ///////////////////////////////////////////////////////////////////////////
    // Class definition

    /*! @class Mesh
        @brief Storage and manipulation of object data.
    */
    class ATO_CORE_EXPORT Mesh
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Static member declarations

      static const core::Float::RGB DefaultColor[6];


      /////////////////////////////////////////////////////////////////////////
      // Inner class definitions

      /*! @struct Material
       *  @brief Definition of guidelines for an object's interaction with
       *         light energy.
       */
      struct Material
      {
        enum Type
          {
            Ambient = 0,
            Diffuse,
            Specular,
            Transmissive,
            Emissive,
            ntypes
          };

        Material(const string& = "unnamed");

        bool read(istream&);
        bool write(ostream&) const;

        friend ATO_CORE_EXPORT istream& operator>>(istream& in, Material&);
        friend ATO_CORE_EXPORT ostream& operator<<(ostream& out, const Material&);


        ///////////////////////////////////////////////////////////////////////
        // Data members

        string m_name;

        core::Float::RGB m_rgb[ntypes];

        string m_filename[ntypes];


        ///////////////////////////////////////////////////////////////////////
        // NOTE(cpg) - Materials share Texture objects, where possible, so
        //             although each tex[i] is heap-allocated, no one Material
        //             owns the Texture and thus no Material is responsible for
        //             its destruction; that's the responsibility of the Mesh

        // XXX(cpg) - rewrite so that each Material has per-type texture IDs,
        //            and make Mesh responsible for managing actual Texture
        //            objects

        Texture* m_tex[ntypes];

        float m_uscale[ntypes];
        float m_vscale[ntypes];

        float m_ns;  // shininess, "Ns" in OBJ
        float m_ni;  // index of refraction, "Ni" or "R0" in OBJ
        float m_tr;  // surface alpha, "Tr" or "d" in OBJ
      };

      /*! @struct Triangle
       *  @brief Storage of triangular shapes.
       */
      struct Triangle
      {
        Triangle(uint, uint, uint, uint, uint = 0);
        Triangle();

        bool read(istream&);
        bool write(ostream&) const;

        friend ATO_CORE_EXPORT istream& operator>>(istream& in, Triangle&);
        friend ATO_CORE_EXPORT ostream& operator<<(ostream& out, const Triangle&);


        ///////////////////////////////////////////////////////////////////////
        // Data members

        uint m_vID[3];
        uint m_mID;
        uint m_oID;
      };

      /*! @struct Vertex
       *  @brief Storage of object's vertices
       */
      struct Vertex
      {
        static const int Undefined;

        Vertex(const Point&, const Vector&, const TexCoord&);
        Vertex();

        bool read(istream&);
        bool write(ostream&) const;

        friend ATO_CORE_EXPORT istream& operator>>(istream& in, Vertex&);
        friend ATO_CORE_EXPORT ostream& operator<<(ostream&, const Vertex&);


        ///////////////////////////////////////////////////////////////////////
        // Data members

        Point    m_p;
        Vector   m_n;
        TexCoord m_t;
      };


      /////////////////////////////////////////////////////////////////////////
      // Exception class

      class InvalidMesh : public Exception
      {
      public:
        InvalidMesh(const string& msg) :
          Exception(msg)
        {
          // no-op
        }

        virtual ~InvalidMesh() throw()
        {
          // no-op
        }

      private:
        // none
      };


      /////////////////////////////////////////////////////////////////////////
      // Constructors, destructor, & assignment operator

      Mesh(const string&);
      Mesh(const Mesh&);
      Mesh();

      ~Mesh();

      Mesh& operator=(const Mesh&);


      /////////////////////////////////////////////////////////////////////////
      // Accessors

      const BBox&                  bounds()    const;
      const vector<Material>&      materials() const;
      const vector<Triangle>&      triangles() const;
      const vector<Vertex>&        vertices()  const;
      const map<string, Texture*>& textures()  const;


      /////////////////////////////////////////////////////////////////////////
      // Mutators

      // XXX(cpg) - mutator form of bounds as added simply to support
      //            mesh transformations in NASA mirror alignment
      //            tool; find a better solution
      BBox&             bounds();
      vector<Material>& materials();
      vector<Triangle>& triangles();
      vector<Vertex>&   vertices();


      /////////////////////////////////////////////////////////////////////////
      // Geometry processing

      void load(const string&);


      /////////////////////////////////////////////////////////////////////////
      // Mesh persistence

      bool read(istream&);
      bool write(ostream&) const;


    protected:

      /*! @struct Sphere
          @brief Storage of spherical shapes.
      */
      struct Sphere
      {
        Sphere(float, float, float, float, uint);


        ///////////////////////////////////////////////////////////////////////
        // Data members

        Point m_c;
        float m_r;
        uint  m_mID;
      };


      /////////////////////////////////////////////////////////////////////////
      // Helpers

      void loadOBJ(const string&);
      void loadMTL(      vector<Material>&,
                         map<string, int>&,
                         string&,
                   const string&);

      void loadML(const string&);
      void loadML(ifstream&);

      void loadIW(const string&);

      void loadRF(const string&);

      Texture* genTexture(const Material&, const Material::Type);


      /////////////////////////////////////////////////////////////////////////
      // Data members

      BBox m_b;

      vector<Material> m_m;
      vector<Triangle> m_t;
      vector<Vertex>   m_v;


      /////////////////////////////////////////////////////////////////////////
      // NOTE(cpg) - this table maps texture filenames or string IDs to
      //             heap-allocated Texture structures.  Texture filenames are
      //             straightforward, as in "mytexture.ppm"; Texture string IDs
      //             are of the form:
      //
      //               rgb <f> <f> <f>
      //
      //             and are generated for Textures that store a single RGB
      //             value. So, entries in this table can be indexed as follows:
      //
      //               tmap["mytexture.ppm"]   --> Texture("mytexture.ppm");
      //               tmap["rgb <f> <f> <f>"] --> Texture(CoreF::RGB(f, f, f))

      map<string, Texture*> m_tmap;

      uint m_nbytes;
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member function definitions

    inline const BBox& Mesh::bounds() const
    {
      return m_b;
    }

    inline const vector<Mesh::Material>& Mesh::materials() const
    {
      return m_m;
    }

    inline const vector<Mesh::Triangle>& Mesh::triangles() const
    {
      return m_t;
    }

    inline const vector<Mesh::Vertex>& Mesh::vertices() const
    {
      return m_v;
    }

    inline const map<string, Texture*>& Mesh::textures() const
    {
      return m_tmap;
    }

    inline BBox& Mesh::bounds()
    {
      return m_b;
    }

    inline vector<Mesh::Material>& Mesh::materials()
    {
      return m_m;
    }

    inline vector<Mesh::Triangle>& Mesh::triangles()
    {
      return m_t;
    }

    inline vector<Mesh::Vertex>& Mesh::vertices()
    {
      return m_v;
    }

  } // namespace core

} // namespace ato

#endif // core_Mesh_h
