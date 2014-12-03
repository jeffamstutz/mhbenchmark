
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012, 2013, Christiaan Gribble <cgribble[]rtvtk org>
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
 *  * Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
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


#ifndef core_ColorMap_t
#define core_ColorMap_t

#include <fstream>
#include <string>
#include <vector>

#include <core/ato_coreExports.h>
#include <core/RGB.t>

#include <math/Helpers.t>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::ifstream;
    using std::string;
    using std::vector;

    using math::Clamp;


    ///////////////////////////////////////////////////////////////////////////
    // Class template definition

    template <typename T>
    class ColorMap
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Enumerated types

      enum Type
        {
          CoolWarm033,
          CoolWarm257,
          UserDefined
        };


      /////////////////////////////////////////////////////////////////////////
      // Constructor

      ColorMap(const Type& type = CoolWarm033);


      /////////////////////////////////////////////////////////////////////////
      // Accessors

      size_t size() const;


      /////////////////////////////////////////////////////////////////////////
      // Core functionality

      bool load(const string&);

      coreT::RGB<T> lookup(float) const;

    private:

      /////////////////////////////////////////////////////////////////////////
      // Helper functions

      void install(const Type&);
      void install(const coreT::RGB<T>*, uint);


      /////////////////////////////////////////////////////////////////////////
      // Data members

      vector<coreT::RGB<T> > m_cmap;

      float m_nint;
      float m_iinv;
      float m_winv;


      /////////////////////////////////////////////////////////////////////////
      // Static members

      static const coreT::RGB<T> CoolWarmData033[ 33];
      static const coreT::RGB<T> CoolWarmData257[257];
    };


    template<typename T>
    ColorMap<T>::ColorMap(const Type& type) :
      m_nint(0),
      m_iinv(0),
      m_winv(0)
    {
      install(type);
    }

    template<typename T>
    size_t ColorMap<T>::size() const
    {
      return m_cmap.size();
    }

    template<typename T>
    bool ColorMap<T>::load(const string& filename)
    {
      ifstream fin(filename.c_str());
      if (!fin.is_open())
      {
        // cout << "Failed to open \"" << filename << "\" for reading" << endl;
        return false;
      }

      T v;
      coreT::RGB<T> c;
      while (fin >> v >> c)
        m_cmap.push_back(c);

      fin.close();

      m_nint = static_cast<float>(size() - 1);
      m_iinv = 1.f/m_nint;
      m_winv = m_nint;

      /*
      cout << endl;
      cout << "Loaded " << cmap.size() << " values from \"" << filename << "\""
           << endl;
      cout << "  m_nint  = " << m_nint << endl;
      cout << "  width = " << 1.f/m_winv << endl;
      cout << "  m_winv  = " << m_winv << endl;
      cout << endl;
      */

      return true;
    }

    template<typename T>
    coreT::RGB<T> ColorMap<T>::lookup(float v) const
    {
      const float x = Clamp(v, 0.f, 1.f);
      const uint  i = static_cast<uint>(x*m_nint);
      const uint  j = (i >= size() ? i : i+1);
      const float d = (x - i*m_iinv)*m_winv;
      /*
      // const CoreF::RGB r = (1.f - d)*cmap[i] + d*cmap[j];
      const CoreF::RGB r = cmap[i] + d*(cmap[j] - cmap[i]);

      cout << endl;
      cout << endl;
      cout << "  v  = " << v << endl;
      cout << "  i  = " << i << endl;
      cout << "  j  = " << j << endl;
      cout << "  ti = " << v - i/static_cast<float>(m_nint) << endl;
      cout << "  tj = " << j/static_cast<float>(m_nint) - v << endl;
      cout << "  d  = " << d << endl;
      cout << "  wi = " << 1.f - d << endl;
      cout << "  wj = " << d << endl;
      cout << "  r  = " << 1.f - d << "*cmap[" << i << "] + " << d << "*cmap["
           << j << "]" << endl;
      cout << "  r  = " << r << endl;

      return r;
      */

      // Operation count
      //   1-  3* 3+ 3* --> 4+/- and 6*
      // return (1.f - d)*cmap[i] + d*cmap[j];

      // Operation count
      //   3+ 3* 3- --> 6+/- and 3*
      return m_cmap[i] + d*(m_cmap[j] - m_cmap[i]);
    }


    ///////////////////////////////////////////////////////////////////////////
    // Helper functions

    template<typename T>
    void ColorMap<T>::install(const Type& type)
    {
            uint           nvals = 0;
      const coreT::RGB<T>* data  = 0;
      switch(type)
      {
      case CoolWarm033:
        nvals = 33;
        data  = CoolWarmData033;

        break;

      case CoolWarm257:
        nvals = 257;
        data  = CoolWarmData257;

        break;

      default:
        // NOTE(cpg) - Either we have a type mismatch or we are expecting user
        //             to invoke load(...) on a valid color map data file
        return;

        break;
      }

      install(data, nvals);
    }

    template<typename T>
    void ColorMap<T>::install(const coreT::RGB<T>* data, uint nvals)
    {
      for (uint i = 0; i < nvals; ++i)
        m_cmap.push_back(data[i]);

      m_nint = static_cast<float>(size() - 1);
      m_iinv = 1.f/m_nint;
      m_winv = m_nint;
    }


    ///////////////////////////////////////////////////////////////////////////
    // Static members

#ifdef WIN32
#ifdef ATO_CORE_EXPORT
    template<>
    const core::Float::RGB ColorMap<float>::CoolWarmData033[33] =
      {
        core::Float::RGB(0.229806f, 0.298718f,  0.753683f),
        core::Float::RGB(0.266234f, 0.353095f,  0.801467f),
        core::Float::RGB(0.303869f, 0.406535f,  0.844959f),
        core::Float::RGB(0.342804f, 0.458758f,  0.883726f),
        core::Float::RGB(0.383013f, 0.509419f,  0.917388f),
        core::Float::RGB(0.42437f,  0.558148f,  0.94562f),
        core::Float::RGB(0.466667f, 0.604563f,  0.968155f),
        core::Float::RGB(0.509635f, 0.648281f,  0.984788f),
        core::Float::RGB(0.552953f, 0.688929f,  0.995376f),
        core::Float::RGB(0.596262f, 0.726149f,  0.999836f),
        core::Float::RGB(0.639176f, 0.7596f,    0.998151f),
        core::Float::RGB(0.681291f, 0.788965f,  0.990363f),
        core::Float::RGB(0.722193f, 0.813953f,  0.976575f),
        core::Float::RGB(0.761465f, 0.834303f,  0.956945f),
        core::Float::RGB(0.798692f, 0.849786f,  0.931689f),
        core::Float::RGB(0.833467f, 0.860208f,  0.901069f),
        core::Float::RGB(0.865395f, 0.86541f,   0.865396f),
        core::Float::RGB(0.897787f, 0.848937f,  0.820881f),
        core::Float::RGB(0.924128f, 0.827385f,  0.774508f),
        core::Float::RGB(0.944468f, 0.800927f,  0.726736f),
        core::Float::RGB(0.958853f, 0.769768f,  0.678008f),
        core::Float::RGB(0.967328f, 0.734133f,  0.628752f),
        core::Float::RGB(0.969954f, 0.694267f,  0.579375f),
        core::Float::RGB(0.966811f, 0.650421f,  0.530264f),
        core::Float::RGB(0.958003f, 0.602842f,  0.481776f),
        core::Float::RGB(0.943661f, 0.551751f,  0.434244f),
        core::Float::RGB(0.923945f, 0.497309f,  0.38797f),
        core::Float::RGB(0.899046f, 0.439559f,  0.34323f),
        core::Float::RGB(0.869187f, 0.378313f,  0.300267f),
        core::Float::RGB(0.834621f, 0.312874f,  0.259301f),
        core::Float::RGB(0.795632f, 0.241284f,  0.220526f),
        core::Float::RGB(0.752535f, 0.157246f,  0.184115f),
        core::Float::RGB(0.705673f, 0.0155562f, 0.150233f)
      };

    template<>
    const core::Float::RGB ColorMap<float>::CoolWarmData257[257] =
      {
        core::Float::RGB(0.229806f, 0.298718f,  0.753683f),
        core::Float::RGB(0.2343f,   0.305559f,  0.759875f),
        core::Float::RGB(0.23881f,  0.312388f,  0.766006f),
        core::Float::RGB(0.243337f, 0.319205f,  0.772075f),
        core::Float::RGB(0.24788f,  0.32601f,   0.778082f),
        core::Float::RGB(0.252441f, 0.332801f,  0.784026f),
        core::Float::RGB(0.25702f,  0.339579f,  0.789905f),
        core::Float::RGB(0.261618f, 0.346344f,  0.795719f),
        core::Float::RGB(0.266234f, 0.353095f,  0.801467f),
        core::Float::RGB(0.270869f, 0.359831f,  0.807147f),
        core::Float::RGB(0.275524f, 0.366552f,  0.81276f),
        core::Float::RGB(0.280198f, 0.373258f,  0.818304f),
        core::Float::RGB(0.284892f, 0.379948f,  0.823777f),
        core::Float::RGB(0.289605f, 0.386621f,  0.829181f),
        core::Float::RGB(0.29434f,  0.393277f,  0.834513f),
        core::Float::RGB(0.299094f, 0.399915f,  0.839772f),
        core::Float::RGB(0.303869f, 0.406535f,  0.844959f),
        core::Float::RGB(0.308664f, 0.413136f,  0.850071f),
        core::Float::RGB(0.31348f,  0.419718f,  0.855109f),
        core::Float::RGB(0.318316f, 0.426279f,  0.860072f),
        core::Float::RGB(0.323173f, 0.432819f,  0.864958f),
        core::Float::RGB(0.328051f, 0.439338f,  0.869767f),
        core::Float::RGB(0.332948f, 0.445834f,  0.874499f),
        core::Float::RGB(0.337866f, 0.452308f,  0.879152f),
        core::Float::RGB(0.342804f, 0.458758f,  0.883726f),
        core::Float::RGB(0.347763f, 0.465183f,  0.88822f),
        core::Float::RGB(0.352741f, 0.471583f,  0.892634f),
        core::Float::RGB(0.357738f, 0.477958f,  0.896966f),
        core::Float::RGB(0.362756f, 0.484306f,  0.901217f),
        core::Float::RGB(0.367792f, 0.490627f,  0.905385f),
        core::Float::RGB(0.372847f, 0.49692f,   0.90947f),
        core::Float::RGB(0.377921f, 0.503184f,  0.913471f),
        core::Float::RGB(0.383013f, 0.509419f,  0.917388f),
        core::Float::RGB(0.388124f, 0.515624f,  0.92122f),
        core::Float::RGB(0.393252f, 0.521797f,  0.924966f),
        core::Float::RGB(0.398397f, 0.527939f,  0.928627f),
        core::Float::RGB(0.403559f, 0.534049f,  0.9322f),
        core::Float::RGB(0.408738f, 0.540125f,  0.935687f),
        core::Float::RGB(0.413933f, 0.546168f,  0.939086f),
        core::Float::RGB(0.419144f, 0.552176f,  0.942397f),
        core::Float::RGB(0.42437f,  0.558148f,  0.94562f),
        core::Float::RGB(0.42961f,  0.564084f,  0.948753f),
        core::Float::RGB(0.434865f, 0.569984f,  0.951797f),
        core::Float::RGB(0.440134f, 0.575845f,  0.95475f),
        core::Float::RGB(0.445416f, 0.581669f,  0.957614f),
        core::Float::RGB(0.450711f, 0.587453f,  0.960386f),
        core::Float::RGB(0.456018f, 0.593197f,  0.963067f),
        core::Float::RGB(0.461337f, 0.5989f,    0.965657f),
        core::Float::RGB(0.466667f, 0.604563f,  0.968155f),
        core::Float::RGB(0.472008f, 0.610183f,  0.97056f),
        core::Float::RGB(0.477358f, 0.61576f,   0.972873f),
        core::Float::RGB(0.482718f, 0.621293f,  0.975093f),
        core::Float::RGB(0.488086f, 0.626782f,  0.97722f),
        core::Float::RGB(0.493463f, 0.632226f,  0.979253f),
        core::Float::RGB(0.498847f, 0.637625f,  0.981192f),
        core::Float::RGB(0.504238f, 0.642976f,  0.983037f),
        core::Float::RGB(0.509635f, 0.648281f,  0.984788f),
        core::Float::RGB(0.515038f, 0.653537f,  0.986444f),
        core::Float::RGB(0.520445f, 0.658745f,  0.988006f),
        core::Float::RGB(0.525857f, 0.663903f,  0.989473f),
        core::Float::RGB(0.531272f, 0.669012f,  0.990844f),
        core::Float::RGB(0.53669f,  0.674069f,  0.99212f),
        core::Float::RGB(0.54211f,  0.679075f,  0.993301f),
        core::Float::RGB(0.547531f, 0.684029f,  0.994386f),
        core::Float::RGB(0.552953f, 0.688929f,  0.995376f),
        core::Float::RGB(0.558375f, 0.693776f,  0.996269f),
        core::Float::RGB(0.563796f, 0.698569f,  0.997067f),
        core::Float::RGB(0.569215f, 0.703307f,  0.997769f),
        core::Float::RGB(0.574632f, 0.70799f,   0.998374f),
        core::Float::RGB(0.580046f, 0.712616f,  0.998884f),
        core::Float::RGB(0.585456f, 0.717185f,  0.999298f),
        core::Float::RGB(0.590862f, 0.721696f,  0.999615f),
        core::Float::RGB(0.596262f, 0.726149f,  0.999836f),
        core::Float::RGB(0.601656f, 0.730543f,  0.999961f),
        core::Float::RGB(0.607043f, 0.734878f,  0.99999f),
        core::Float::RGB(0.612423f, 0.739152f,  0.999924f),
        core::Float::RGB(0.617794f, 0.743366f,  0.999761f),
        core::Float::RGB(0.623155f, 0.747518f,  0.999502f),
        core::Float::RGB(0.628507f, 0.751608f,  0.999147f),
        core::Float::RGB(0.633847f, 0.755636f,  0.998697f),
        core::Float::RGB(0.639176f, 0.7596f,    0.998151f),
        core::Float::RGB(0.644493f, 0.7635f,    0.99751f),
        core::Float::RGB(0.649796f, 0.767336f,  0.996773f),
        core::Float::RGB(0.655085f, 0.771107f,  0.995942f),
        core::Float::RGB(0.660359f, 0.774812f,  0.995015f),
        core::Float::RGB(0.665618f, 0.778451f,  0.993994f),
        core::Float::RGB(0.67086f,  0.782023f,  0.992878f),
        core::Float::RGB(0.676085f, 0.785528f,  0.991668f),
        core::Float::RGB(0.681291f, 0.788965f,  0.990363f),
        core::Float::RGB(0.686479f, 0.792333f,  0.988965f),
        core::Float::RGB(0.691647f, 0.795633f,  0.987473f),
        core::Float::RGB(0.696794f, 0.798863f,  0.985888f),
        core::Float::RGB(0.70192f,  0.802023f,  0.98421f),
        core::Float::RGB(0.707024f, 0.805112f,  0.98244f),
        core::Float::RGB(0.712104f, 0.808131f,  0.980577f),
        core::Float::RGB(0.717161f, 0.811078f,  0.978621f),
        core::Float::RGB(0.722193f, 0.813953f,  0.976575f),
        core::Float::RGB(0.7272f,   0.816755f,  0.974437f),
        core::Float::RGB(0.73218f,  0.819485f,  0.972207f),
        core::Float::RGB(0.737134f, 0.822141f,  0.969888f),
        core::Float::RGB(0.742059f, 0.824723f,  0.967478f),
        core::Float::RGB(0.746956f, 0.82723f,   0.964978f),
        core::Float::RGB(0.751823f, 0.829663f,  0.962389f),
        core::Float::RGB(0.756659f, 0.832021f,  0.959712f),
        core::Float::RGB(0.761465f, 0.834303f,  0.956945f),
        core::Float::RGB(0.766239f, 0.836509f,  0.954091f),
        core::Float::RGB(0.770979f, 0.838638f,  0.951149f),
        core::Float::RGB(0.775687f, 0.840691f,  0.94812f),
        core::Float::RGB(0.78036f,  0.842666f,  0.945005f),
        core::Float::RGB(0.784997f, 0.844563f,  0.941804f),
        core::Float::RGB(0.789599f, 0.846383f,  0.938517f),
        core::Float::RGB(0.794164f, 0.848124f,  0.935145f),
        core::Float::RGB(0.798692f, 0.849786f,  0.931689f),
        core::Float::RGB(0.803181f, 0.851369f,  0.928149f),
        core::Float::RGB(0.807631f, 0.852873f,  0.924525f),
        core::Float::RGB(0.812041f, 0.854297f,  0.920819f),
        core::Float::RGB(0.816411f, 0.85564f,   0.917031f),
        core::Float::RGB(0.820739f, 0.856904f,  0.913161f),
        core::Float::RGB(0.825024f, 0.858086f,  0.90921f),
        core::Float::RGB(0.829267f, 0.859188f,  0.905179f),
        core::Float::RGB(0.833467f, 0.860208f,  0.901069f),
        core::Float::RGB(0.837621f, 0.861147f,  0.896879f),
        core::Float::RGB(0.841731f, 0.862003f,  0.892611f),
        core::Float::RGB(0.845794f, 0.862778f,  0.888266f),
        core::Float::RGB(0.849811f, 0.86347f,   0.883843f),
        core::Float::RGB(0.85378f,  0.86408f,   0.879344f),
        core::Float::RGB(0.857701f, 0.864606f,  0.874769f),
        core::Float::RGB(0.861573f, 0.86505f,   0.870119f),
        core::Float::RGB(0.865395f, 0.86541f,   0.865396f),
        core::Float::RGB(0.869778f, 0.863634f,  0.859949f),
        core::Float::RGB(0.874064f, 0.861776f,  0.854466f),
        core::Float::RGB(0.878256f, 0.859838f,  0.848949f),
        core::Float::RGB(0.882352f, 0.857818f,  0.843399f),
        core::Float::RGB(0.886353f, 0.855718f,  0.837816f),
        core::Float::RGB(0.890259f, 0.853538f,  0.832201f),
        core::Float::RGB(0.89407f,  0.851277f,  0.826556f),
        core::Float::RGB(0.897787f, 0.848937f,  0.820881f),
        core::Float::RGB(0.901409f, 0.846518f,  0.815176f),
        core::Float::RGB(0.904937f, 0.844019f,  0.809444f),
        core::Float::RGB(0.908371f, 0.841442f,  0.803684f),
        core::Float::RGB(0.91171f,  0.838786f,  0.797898f),
        core::Float::RGB(0.914955f, 0.836052f,  0.792086f),
        core::Float::RGB(0.918107f, 0.83324f,   0.78625f),
        core::Float::RGB(0.921164f, 0.830351f,  0.780391f),
        core::Float::RGB(0.924128f, 0.827385f,  0.774508f),
        core::Float::RGB(0.926997f, 0.824342f,  0.768604f),
        core::Float::RGB(0.929774f, 0.821223f,  0.762679f),
        core::Float::RGB(0.932456f, 0.818028f,  0.756734f),
        core::Float::RGB(0.935045f, 0.814757f,  0.750769f),
        core::Float::RGB(0.937541f, 0.811411f,  0.744786f),
        core::Float::RGB(0.939943f, 0.807991f,  0.738786f),
        core::Float::RGB(0.942253f, 0.804496f,  0.732769f),
        core::Float::RGB(0.944468f, 0.800927f,  0.726736f),
        core::Float::RGB(0.946591f, 0.797285f,  0.720688f),
        core::Float::RGB(0.948621f, 0.79357f,   0.714627f),
        core::Float::RGB(0.950558f, 0.789782f,  0.708552f),
        core::Float::RGB(0.952403f, 0.785922f,  0.702464f),
        core::Float::RGB(0.954154f, 0.78199f,   0.696365f),
        core::Float::RGB(0.955813f, 0.777986f,  0.690256f),
        core::Float::RGB(0.957379f, 0.773912f,  0.684136f),
        core::Float::RGB(0.958853f, 0.769768f,  0.678008f),
        core::Float::RGB(0.960234f, 0.765553f,  0.671871f),
        core::Float::RGB(0.961524f, 0.761269f,  0.665727f),
        core::Float::RGB(0.962721f, 0.756916f,  0.659576f),
        core::Float::RGB(0.963826f, 0.752495f,  0.65342f),
        core::Float::RGB(0.964839f, 0.748005f,  0.647258f),
        core::Float::RGB(0.96576f,  0.743448f,  0.641093f),
        core::Float::RGB(0.96659f,  0.738824f,  0.634924f),
        core::Float::RGB(0.967328f, 0.734133f,  0.628752f),
        core::Float::RGB(0.967975f, 0.729376f,  0.622578f),
        core::Float::RGB(0.96853f,  0.724553f,  0.616404f),
        core::Float::RGB(0.968994f, 0.719665f,  0.610229f),
        core::Float::RGB(0.969368f, 0.714713f,  0.604054f),
        core::Float::RGB(0.96965f,  0.709696f,  0.597881f),
        core::Float::RGB(0.969842f, 0.704616f,  0.591709f),
        core::Float::RGB(0.969943f, 0.699473f,  0.585541f),
        core::Float::RGB(0.969954f, 0.694267f,  0.579375f),
        core::Float::RGB(0.969875f, 0.688998f,  0.573214f),
        core::Float::RGB(0.969706f, 0.683669f,  0.567058f),
        core::Float::RGB(0.969447f, 0.678277f,  0.560908f),
        core::Float::RGB(0.969098f, 0.672826f,  0.554763f),
        core::Float::RGB(0.96866f,  0.667314f,  0.548626f),
        core::Float::RGB(0.968133f, 0.661742f,  0.542497f),
        core::Float::RGB(0.967516f, 0.656111f,  0.536376f),
        core::Float::RGB(0.966811f, 0.650421f,  0.530264f),
        core::Float::RGB(0.966018f, 0.644673f,  0.524162f),
        core::Float::RGB(0.965136f, 0.638867f,  0.51807f),
        core::Float::RGB(0.964166f, 0.633004f,  0.511989f),
        core::Float::RGB(0.963108f, 0.627084f,  0.50592f),
        core::Float::RGB(0.961962f, 0.621107f,  0.499864f),
        core::Float::RGB(0.96073f,  0.615074f,  0.493821f),
        core::Float::RGB(0.95941f,  0.608986f,  0.487791f),
        core::Float::RGB(0.958003f, 0.602842f,  0.481776f),
        core::Float::RGB(0.95651f,  0.596644f,  0.475776f),
        core::Float::RGB(0.954931f, 0.590391f,  0.469791f),
        core::Float::RGB(0.953265f, 0.584084f,  0.463822f),
        core::Float::RGB(0.951514f, 0.577724f,  0.457871f),
        core::Float::RGB(0.949678f, 0.57131f,   0.451936f),
        core::Float::RGB(0.947757f, 0.564843f,  0.44602f),
        core::Float::RGB(0.945751f, 0.558323f,  0.440122f),
        core::Float::RGB(0.943661f, 0.551751f,  0.434244f),
        core::Float::RGB(0.941487f, 0.545127f,  0.428385f),
        core::Float::RGB(0.939229f, 0.53845f,   0.422546f),
        core::Float::RGB(0.936888f, 0.531722f,  0.416728f),
        core::Float::RGB(0.934463f, 0.524942f,  0.410932f),
        core::Float::RGB(0.931957f, 0.518111f,  0.405157f),
        core::Float::RGB(0.929368f, 0.511228f,  0.399405f),
        core::Float::RGB(0.926697f, 0.504294f,  0.393676f),
        core::Float::RGB(0.923945f, 0.497309f,  0.38797f),
        core::Float::RGB(0.921112f, 0.490272f,  0.382289f),
        core::Float::RGB(0.918198f, 0.483183f,  0.376631f),
        core::Float::RGB(0.915204f, 0.476043f,  0.370999f),
        core::Float::RGB(0.91213f,  0.468852f,  0.365392f),
        core::Float::RGB(0.908977f, 0.461608f,  0.359812f),
        core::Float::RGB(0.905745f, 0.454311f,  0.354257f),
        core::Float::RGB(0.902435f, 0.446962f,  0.34873f),
        core::Float::RGB(0.899046f, 0.439559f,  0.34323f),
        core::Float::RGB(0.89558f,  0.432103f,  0.337757f),
        core::Float::RGB(0.892037f, 0.424591f,  0.332313f),
        core::Float::RGB(0.888417f, 0.417024f,  0.326898f),
        core::Float::RGB(0.884721f, 0.4094f,    0.321512f),
        core::Float::RGB(0.88095f,  0.401718f,  0.316155f),
        core::Float::RGB(0.877103f, 0.393978f,  0.310829f),
        core::Float::RGB(0.873182f, 0.386177f,  0.305533f),
        core::Float::RGB(0.869187f, 0.378313f,  0.300267f),
        core::Float::RGB(0.865118f, 0.370386f,  0.295033f),
        core::Float::RGB(0.860976f, 0.362392f,  0.289831f),
        core::Float::RGB(0.856761f, 0.354329f,  0.28466f),
        core::Float::RGB(0.852475f, 0.346195f,  0.279522f),
        core::Float::RGB(0.848117f, 0.337986f,  0.274417f),
        core::Float::RGB(0.843688f, 0.329699f,  0.269345f),
        core::Float::RGB(0.839189f, 0.32133f,   0.264306f),
        core::Float::RGB(0.834621f, 0.312874f,  0.259301f),
        core::Float::RGB(0.829983f, 0.304327f,  0.254331f),
        core::Float::RGB(0.825276f, 0.295681f,  0.249395f),
        core::Float::RGB(0.820502f, 0.28693f,   0.244494f),
        core::Float::RGB(0.81566f,  0.278067f,  0.239628f),
        core::Float::RGB(0.810751f, 0.269082f,  0.234798f),
        core::Float::RGB(0.805777f, 0.259965f,  0.230004f),
        core::Float::RGB(0.800737f, 0.250704f,  0.225247f),
        core::Float::RGB(0.795632f, 0.241284f,  0.220526f),
        core::Float::RGB(0.790463f, 0.231689f,  0.215842f),
        core::Float::RGB(0.78523f,  0.221898f,  0.211195f),
        core::Float::RGB(0.779934f, 0.211889f,  0.206586f),
        core::Float::RGB(0.774576f, 0.201631f,  0.202014f),
        core::Float::RGB(0.769156f, 0.191089f,  0.197481f),
        core::Float::RGB(0.763676f, 0.180217f,  0.192987f),
        core::Float::RGB(0.758135f, 0.168961f,  0.188531f),
        core::Float::RGB(0.752535f, 0.157246f,  0.184115f),
        core::Float::RGB(0.746876f, 0.144975f,  0.179738f),
        core::Float::RGB(0.741158f, 0.132014f,  0.175401f),
        core::Float::RGB(0.735384f, 0.118172f,  0.171104f),
        core::Float::RGB(0.729552f, 0.103159f,  0.166848f),
        core::Float::RGB(0.723665f, 0.0865047f, 0.162632f),
        core::Float::RGB(0.717722f, 0.067344f,  0.158458f),
        core::Float::RGB(0.711724f, 0.0437552f, 0.154324f),
        core::Float::RGB(0.705673f, 0.0155562f, 0.150233f)
      };

    template<>
    const coreT::RGB<uchar> ColorMap<uchar>::CoolWarmData033[33] =
      {
        coreT::RGB<uchar>(59,   76, 192),
        coreT::RGB<uchar>(68,   90, 204),
        coreT::RGB<uchar>(77,  104, 215),
        coreT::RGB<uchar>(87,  117, 225),
        coreT::RGB<uchar>(98,  130, 234),
        coreT::RGB<uchar>(108, 142, 241),
        coreT::RGB<uchar>(119, 154, 247),
        coreT::RGB<uchar>(130, 165, 251),
        coreT::RGB<uchar>(141, 176, 254),
        coreT::RGB<uchar>(152, 185, 255),
        coreT::RGB<uchar>(163, 194, 255),
        coreT::RGB<uchar>(174, 201, 253),
        coreT::RGB<uchar>(184, 208, 249),
        coreT::RGB<uchar>(194, 213, 244),
        coreT::RGB<uchar>(204, 217, 238),
        coreT::RGB<uchar>(213, 219, 230),
        coreT::RGB<uchar>(221, 221, 221),
        coreT::RGB<uchar>(229, 216, 209),
        coreT::RGB<uchar>(236, 211, 197),
        coreT::RGB<uchar>(241, 204, 185),
        coreT::RGB<uchar>(245, 196, 173),
        coreT::RGB<uchar>(247, 187, 160),
        coreT::RGB<uchar>(247, 177, 148),
        coreT::RGB<uchar>(247, 166, 135),
        coreT::RGB<uchar>(244, 154, 123),
        coreT::RGB<uchar>(241, 141, 111),
        coreT::RGB<uchar>(236, 127,  99),
        coreT::RGB<uchar>(229, 112,  88),
        coreT::RGB<uchar>(222,  96,  77),
        coreT::RGB<uchar>(213,  80,  66),
        coreT::RGB<uchar>(203,  62,  56),
        coreT::RGB<uchar>(192,  40,  47),
        coreT::RGB<uchar>(180,   4,  38)
      };

    template<>
    const coreT::RGB<uchar> ColorMap<uchar>::CoolWarmData257[257] =
      {
        coreT::RGB<uchar>( 59,  76, 192),
        coreT::RGB<uchar>( 60,  78, 194),
        coreT::RGB<uchar>( 61,  80, 195),
        coreT::RGB<uchar>( 62,  81, 197),
        coreT::RGB<uchar>( 63,  83, 198),
        coreT::RGB<uchar>( 64,  85, 200),
        coreT::RGB<uchar>( 66,  87, 201),
        coreT::RGB<uchar>( 67,  88, 203),
        coreT::RGB<uchar>( 68,  90, 204),
        coreT::RGB<uchar>( 69,  92, 206),
        coreT::RGB<uchar>( 70,  93, 207),
        coreT::RGB<uchar>( 71,  95, 209),
        coreT::RGB<uchar>( 73,  97, 210),
        coreT::RGB<uchar>( 74,  99, 211),
        coreT::RGB<uchar>( 75, 100, 213),
        coreT::RGB<uchar>( 76, 102, 214),
        coreT::RGB<uchar>( 77, 104, 215),
        coreT::RGB<uchar>( 79, 105, 217),
        coreT::RGB<uchar>( 80, 107, 218),
        coreT::RGB<uchar>( 81, 109, 219),
        coreT::RGB<uchar>( 82, 110, 221),
        coreT::RGB<uchar>( 84, 112, 222),
        coreT::RGB<uchar>( 85, 114, 223),
        coreT::RGB<uchar>( 86, 115, 224),
        coreT::RGB<uchar>( 87, 117, 225),
        coreT::RGB<uchar>( 89, 119, 226),
        coreT::RGB<uchar>( 90, 120, 228),
        coreT::RGB<uchar>( 91, 122, 229),
        coreT::RGB<uchar>( 93, 123, 230),
        coreT::RGB<uchar>( 94, 125, 231),
        coreT::RGB<uchar>( 95, 127, 232),
        coreT::RGB<uchar>( 96, 128, 233),
        coreT::RGB<uchar>( 98, 130, 234),
        coreT::RGB<uchar>( 99, 131, 235),
        coreT::RGB<uchar>(100, 133, 236),
        coreT::RGB<uchar>(102, 135, 237),
        coreT::RGB<uchar>(103, 136, 238),
        coreT::RGB<uchar>(104, 138, 239),
        coreT::RGB<uchar>(106, 139, 239),
        coreT::RGB<uchar>(107, 141, 240),
        coreT::RGB<uchar>(108, 142, 241),
        coreT::RGB<uchar>(110, 144, 242),
        coreT::RGB<uchar>(111, 145, 243),
        coreT::RGB<uchar>(112, 147, 243),
        coreT::RGB<uchar>(114, 148, 244),
        coreT::RGB<uchar>(115, 150, 245),
        coreT::RGB<uchar>(116, 151, 246),
        coreT::RGB<uchar>(118, 153, 246),
        coreT::RGB<uchar>(119, 154, 247),
        coreT::RGB<uchar>(120, 156, 247),
        coreT::RGB<uchar>(122, 157, 248),
        coreT::RGB<uchar>(123, 158, 249),
        coreT::RGB<uchar>(124, 160, 249),
        coreT::RGB<uchar>(126, 161, 250),
        coreT::RGB<uchar>(127, 163, 250),
        coreT::RGB<uchar>(129, 164, 251),
        coreT::RGB<uchar>(130, 165, 251),
        coreT::RGB<uchar>(131, 167, 252),
        coreT::RGB<uchar>(133, 168, 252),
        coreT::RGB<uchar>(134, 169, 252),
        coreT::RGB<uchar>(135, 171, 253),
        coreT::RGB<uchar>(137, 172, 253),
        coreT::RGB<uchar>(138, 173, 253),
        coreT::RGB<uchar>(140, 174, 254),
        coreT::RGB<uchar>(141, 176, 254),
        coreT::RGB<uchar>(142, 177, 254),
        coreT::RGB<uchar>(144, 178, 254),
        coreT::RGB<uchar>(145, 179, 254),
        coreT::RGB<uchar>(147, 181, 255),
        coreT::RGB<uchar>(148, 182, 255),
        coreT::RGB<uchar>(149, 183, 255),
        coreT::RGB<uchar>(151, 184, 255),
        coreT::RGB<uchar>(152, 185, 255),
        coreT::RGB<uchar>(153, 186, 255),
        coreT::RGB<uchar>(155, 187, 255),
        coreT::RGB<uchar>(156, 188, 255),
        coreT::RGB<uchar>(158, 190, 255),
        coreT::RGB<uchar>(159, 191, 255),
        coreT::RGB<uchar>(160, 192, 255),
        coreT::RGB<uchar>(162, 193, 255),
        coreT::RGB<uchar>(163, 194, 255),
        coreT::RGB<uchar>(164, 195, 254),
        coreT::RGB<uchar>(166, 196, 254),
        coreT::RGB<uchar>(167, 197, 254),
        coreT::RGB<uchar>(168, 198, 254),
        coreT::RGB<uchar>(170, 199, 253),
        coreT::RGB<uchar>(171, 199, 253),
        coreT::RGB<uchar>(172, 200, 253),
        coreT::RGB<uchar>(174, 201, 253),
        coreT::RGB<uchar>(175, 202, 252),
        coreT::RGB<uchar>(176, 203, 252),
        coreT::RGB<uchar>(178, 204, 251),
        coreT::RGB<uchar>(179, 205, 251),
        coreT::RGB<uchar>(180, 205, 251),
        coreT::RGB<uchar>(182, 206, 250),
        coreT::RGB<uchar>(183, 207, 250),
        coreT::RGB<uchar>(184, 208, 249),
        coreT::RGB<uchar>(185, 208, 248),
        coreT::RGB<uchar>(187, 209, 248),
        coreT::RGB<uchar>(188, 210, 247),
        coreT::RGB<uchar>(189, 210, 247),
        coreT::RGB<uchar>(190, 211, 246),
        coreT::RGB<uchar>(192, 212, 245),
        coreT::RGB<uchar>(193, 212, 245),
        coreT::RGB<uchar>(194, 213, 244),
        coreT::RGB<uchar>(195, 213, 243),
        coreT::RGB<uchar>(197, 214, 243),
        coreT::RGB<uchar>(198, 214, 242),
        coreT::RGB<uchar>(199, 215, 241),
        coreT::RGB<uchar>(200, 215, 240),
        coreT::RGB<uchar>(201, 216, 239),
        coreT::RGB<uchar>(203, 216, 238),
        coreT::RGB<uchar>(204, 217, 238),
        coreT::RGB<uchar>(205, 217, 237),
        coreT::RGB<uchar>(206, 217, 236),
        coreT::RGB<uchar>(207, 218, 235),
        coreT::RGB<uchar>(208, 218, 234),
        coreT::RGB<uchar>(209, 219, 233),
        coreT::RGB<uchar>(210, 219, 232),
        coreT::RGB<uchar>(211, 219, 231),
        coreT::RGB<uchar>(213, 219, 230),
        coreT::RGB<uchar>(214, 220, 229),
        coreT::RGB<uchar>(215, 220, 228),
        coreT::RGB<uchar>(216, 220, 227),
        coreT::RGB<uchar>(217, 220, 225),
        coreT::RGB<uchar>(218, 220, 224),
        coreT::RGB<uchar>(219, 220, 223),
        coreT::RGB<uchar>(220, 221, 222),
        coreT::RGB<uchar>(221, 221, 221),
        coreT::RGB<uchar>(222, 220, 219),
        coreT::RGB<uchar>(223, 220, 218),
        coreT::RGB<uchar>(224, 219, 216),
        coreT::RGB<uchar>(225, 219, 215),
        coreT::RGB<uchar>(226, 218, 214),
        coreT::RGB<uchar>(227, 218, 212),
        coreT::RGB<uchar>(228, 217, 211),
        coreT::RGB<uchar>(229, 216, 209),
        coreT::RGB<uchar>(230, 216, 208),
        coreT::RGB<uchar>(231, 215, 206),
        coreT::RGB<uchar>(232, 215, 205),
        coreT::RGB<uchar>(232, 214, 203),
        coreT::RGB<uchar>(233, 213, 202),
        coreT::RGB<uchar>(234, 212, 200),
        coreT::RGB<uchar>(235, 212, 199),
        coreT::RGB<uchar>(236, 211, 197),
        coreT::RGB<uchar>(236, 210, 196),
        coreT::RGB<uchar>(237, 209, 194),
        coreT::RGB<uchar>(238, 209, 193),
        coreT::RGB<uchar>(238, 208, 191),
        coreT::RGB<uchar>(239, 207, 190),
        coreT::RGB<uchar>(240, 206, 188),
        coreT::RGB<uchar>(240, 205, 187),
        coreT::RGB<uchar>(241, 204, 185),
        coreT::RGB<uchar>(241, 203, 184),
        coreT::RGB<uchar>(242, 202, 182),
        coreT::RGB<uchar>(242, 201, 181),
        coreT::RGB<uchar>(243, 200, 179),
        coreT::RGB<uchar>(243, 199, 178),
        coreT::RGB<uchar>(244, 198, 176),
        coreT::RGB<uchar>(244, 197, 174),
        coreT::RGB<uchar>(245, 196, 173),
        coreT::RGB<uchar>(245, 195, 171),
        coreT::RGB<uchar>(245, 194, 170),
        coreT::RGB<uchar>(245, 193, 168),
        coreT::RGB<uchar>(246, 192, 167),
        coreT::RGB<uchar>(246, 191, 165),
        coreT::RGB<uchar>(246, 190, 163),
        coreT::RGB<uchar>(246, 188, 162),
        coreT::RGB<uchar>(247, 187, 160),
        coreT::RGB<uchar>(247, 186, 159),
        coreT::RGB<uchar>(247, 185, 157),
        coreT::RGB<uchar>(247, 184, 156),
        coreT::RGB<uchar>(247, 182, 154),
        coreT::RGB<uchar>(247, 181, 152),
        coreT::RGB<uchar>(247, 180, 151),
        coreT::RGB<uchar>(247, 178, 149),
        coreT::RGB<uchar>(247, 177, 148),
        coreT::RGB<uchar>(247, 176, 146),
        coreT::RGB<uchar>(247, 174, 145),
        coreT::RGB<uchar>(247, 173, 143),
        coreT::RGB<uchar>(247, 172, 141),
        coreT::RGB<uchar>(247, 170, 140),
        coreT::RGB<uchar>(247, 169, 138),
        coreT::RGB<uchar>(247, 167, 137),
        coreT::RGB<uchar>(247, 166, 135),
        coreT::RGB<uchar>(246, 164, 134),
        coreT::RGB<uchar>(246, 163, 132),
        coreT::RGB<uchar>(246, 161, 131),
        coreT::RGB<uchar>(246, 160, 129),
        coreT::RGB<uchar>(245, 158, 127),
        coreT::RGB<uchar>(245, 157, 126),
        coreT::RGB<uchar>(245, 155, 124),
        coreT::RGB<uchar>(244, 154, 123),
        coreT::RGB<uchar>(244, 152, 121),
        coreT::RGB<uchar>(244, 151, 120),
        coreT::RGB<uchar>(243, 149, 118),
        coreT::RGB<uchar>(243, 147, 117),
        coreT::RGB<uchar>(242, 146, 115),
        coreT::RGB<uchar>(242, 144, 114),
        coreT::RGB<uchar>(241, 142, 112),
        coreT::RGB<uchar>(241, 141, 111),
        coreT::RGB<uchar>(240, 139, 109),
        coreT::RGB<uchar>(240, 137, 108),
        coreT::RGB<uchar>(239, 136, 106),
        coreT::RGB<uchar>(238, 134, 105),
        coreT::RGB<uchar>(238, 132, 103),
        coreT::RGB<uchar>(237, 130, 102),
        coreT::RGB<uchar>(236, 129, 100),
        coreT::RGB<uchar>(236, 127,  99),
        coreT::RGB<uchar>(235, 125,  97),
        coreT::RGB<uchar>(234, 123,  96),
        coreT::RGB<uchar>(233, 121,  95),
        coreT::RGB<uchar>(233, 120,  93),
        coreT::RGB<uchar>(232, 118,  92),
        coreT::RGB<uchar>(231, 116,  90),
        coreT::RGB<uchar>(230, 114,  89),
        coreT::RGB<uchar>(229, 112,  88),
        coreT::RGB<uchar>(228, 110,  86),
        coreT::RGB<uchar>(227, 108,  85),
        coreT::RGB<uchar>(227, 106,  83),
        coreT::RGB<uchar>(226, 104,  82),
        coreT::RGB<uchar>(225, 102,  81),
        coreT::RGB<uchar>(224, 100,  79),
        coreT::RGB<uchar>(223,  98,  78),
        coreT::RGB<uchar>(222,  96,  77),
        coreT::RGB<uchar>(221,  94,  75),
        coreT::RGB<uchar>(220,  92,  74),
        coreT::RGB<uchar>(218,  90,  73),
        coreT::RGB<uchar>(217,  88,  71),
        coreT::RGB<uchar>(216,  86,  70),
        coreT::RGB<uchar>(215,  84,  69),
        coreT::RGB<uchar>(214,  82,  67),
        coreT::RGB<uchar>(213,  80,  66),
        coreT::RGB<uchar>(212,  78,  65),
        coreT::RGB<uchar>(210,  75,  64),
        coreT::RGB<uchar>(209,  73,  62),
        coreT::RGB<uchar>(208,  71,  61),
        coreT::RGB<uchar>(207,  69,  60),
        coreT::RGB<uchar>(205,  66,  59),
        coreT::RGB<uchar>(204,  64,  57),
        coreT::RGB<uchar>(203,  62,  56),
        coreT::RGB<uchar>(202,  59,  55),
        coreT::RGB<uchar>(200,  57,  54),
        coreT::RGB<uchar>(199,  54,  53),
        coreT::RGB<uchar>(198,  51,  52),
        coreT::RGB<uchar>(196,  49,  50),
        coreT::RGB<uchar>(195,  46,  49),
        coreT::RGB<uchar>(193,  43,  48),
        coreT::RGB<uchar>(192,  40,  47),
        coreT::RGB<uchar>(190,  37,  46),
        coreT::RGB<uchar>(189,  34,  45),
        coreT::RGB<uchar>(188,  30,  44),
        coreT::RGB<uchar>(186,  26,  43),
        coreT::RGB<uchar>(185,  22,  41),
        coreT::RGB<uchar>(183,  17,  40),
        coreT::RGB<uchar>(181,  11,  39),
        coreT::RGB<uchar>(180,   4,  38)
      };
#endif // CORE_ATO_EXPORT
#endif // WIN32

    namespace Float
    {

      /////////////////////////////////////////////////////////////////////////
      // Type definitions

      typedef core::ColorMap<float> ColorMap;

    } // namespace Float

  } // namespace core

} // namespace ato

#endif // core_ColorMap_t
