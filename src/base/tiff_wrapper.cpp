/*
 * @Author: Han
 * @Date: 2018-05-10 19:19:39
 * 封装 tiff 函数
 */
#include <base/tiff_wrapper.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <tiffio.h>
namespace h2o {

void icvCvt_BGR2RGB_8u_C3R(const uchar *bgr, int bgr_step, uchar *rgb, int rgb_step, cv::Size size) {
    int i;
    for (; size.height--;) {
        for (i = 0; i < size.width; i++, bgr += 3, rgb += 3) {
            uchar t0 = bgr[0], t1 = bgr[1], t2 = bgr[2];
            rgb[2] = t0;
            rgb[1] = t1;
            rgb[0] = t2;
        }
        bgr += bgr_step - size.width * 3;
        rgb += rgb_step - size.width * 3;
    }
}

void icvCvt_BGR2RGB_16u_C3R(const ushort *bgr, int bgr_step, ushort *rgb, int rgb_step, cv::Size size) {
    int i;
    for (; size.height--;) {
        for (i = 0; i < size.width; i++, bgr += 3, rgb += 3) {
            ushort t0 = bgr[0], t1 = bgr[1], t2 = bgr[2];
            rgb[2] = t0;
            rgb[1] = t1;
            rgb[0] = t2;
        }
        bgr += bgr_step - size.width * 3;
        rgb += rgb_step - size.width * 3;
    }
}

void icvCvt_BGRA2RGBA_8u_C4R(const uchar *bgra, int bgra_step, uchar *rgba, int rgba_step, cv::Size size) {
    int i;
    for (; size.height--;) {
        for (i = 0; i < size.width; i++, bgra += 4, rgba += 4) {
            uchar t0 = bgra[0], t1 = bgra[1];
            uchar t2 = bgra[2], t3 = bgra[3];
            rgba[0] = t2;
            rgba[1] = t1;
            rgba[2] = t0;
            rgba[3] = t3;
        }
        bgra += bgra_step - size.width * 4;
        rgba += rgba_step - size.width * 4;
    }
}

void icvCvt_BGRA2RGBA_16u_C4R(const ushort *bgra, int bgra_step, ushort *rgba, int rgba_step, cv::Size size) {
    int i;
    for (; size.height--;) {
        for (i = 0; i < size.width; i++, bgra += 4, rgba += 4) {
            ushort t0 = bgra[0], t1 = bgra[1];
            ushort t2 = bgra[2], t3 = bgra[3];

            rgba[0] = t2;
            rgba[1] = t1;
            rgba[2] = t0;
            rgba[3] = t3;
        }
        bgra += bgra_step / sizeof(bgra[0]) - size.width * 4;
        rgba += rgba_step / sizeof(rgba[0]) - size.width * 4;
    }
}

class TiffEncoderBufHelper {
public:
    TiffEncoderBufHelper(std::vector<uint8_t> *buf) : m_buf(buf), m_buf_pos(0) {}

    TIFF *open() {
        return TIFFClientOpen("", "w", reinterpret_cast<thandle_t>(this), &TiffEncoderBufHelper::read,
                              &TiffEncoderBufHelper::write, &TiffEncoderBufHelper::seek, &TiffEncoderBufHelper::close,
                              &TiffEncoderBufHelper::size,
                              /*map=*/0, /*unmap=*/0);
    }

    static tmsize_t read(thandle_t handle, void *buffer, tmsize_t n) {
        TiffEncoderBufHelper *helper = reinterpret_cast<TiffEncoderBufHelper *>(handle);
        const std::vector<uint8_t> *buf = helper->m_buf;
        tmsize_t pos = helper->m_buf_pos;

        memcpy(buffer, buf->data() + pos, n);
        helper->m_buf_pos += n;
        return n;
    }

    static tmsize_t write(thandle_t handle, void *buffer, tmsize_t n) {
        TiffEncoderBufHelper *helper = reinterpret_cast<TiffEncoderBufHelper *>(handle);
        size_t begin = (size_t)helper->m_buf_pos;
        size_t end = begin + n;
        if (helper->m_buf->size() < end) {
            helper->m_buf->resize(end);
        }
        memcpy(&(*helper->m_buf)[begin], buffer, n);
        helper->m_buf_pos = end;
        return n;
    }

    static toff_t seek(thandle_t handle, toff_t offset, int whence) {
        TiffEncoderBufHelper *helper = reinterpret_cast<TiffEncoderBufHelper *>(handle);
        const toff_t size = helper->m_buf->size();
        toff_t new_pos = helper->m_buf_pos;
        switch (whence) {
        case SEEK_SET:
            new_pos = offset;
            break;
        case SEEK_CUR:
            new_pos += offset;
            break;
        case SEEK_END:
            new_pos = size + offset;
            break;
        }
        helper->m_buf_pos = new_pos;
        return new_pos;
    }

    static toff_t size(thandle_t handle) {
        TiffEncoderBufHelper *helper = reinterpret_cast<TiffEncoderBufHelper *>(handle);
        return helper->m_buf->size();
    }

    static int close(thandle_t handle) { return 0; }

private:
    std::vector<uint8_t> *m_buf;
    toff_t m_buf_pos;
};

static void readParam(const std::vector<int> &params, int key, int &value) {
    for (size_t i = 0; i + 1 < params.size(); i += 2)
        if (params[i] == key) {
            value = params[i + 1];
            break;
        }
}

std::vector<uint8_t> tiff_encode(const cv::Mat &mat, TiffCompression compression) {

    int channels = mat.channels();
    int width = mat.cols, height = mat.rows;
    int depth = mat.depth();

    int bitsPerChannel = -1;
    switch (depth) {
    case CV_8U: {
        bitsPerChannel = 8;
        break;
    }
    case CV_16U: {
        bitsPerChannel = 16;
        break;
    }
    default: { return std::vector<uint8_t>(); }
    }

    const int bitsPerByte = 8;
    size_t fileStep = (width * channels * bitsPerChannel) / bitsPerByte;

    int rowsPerStrip = (int)((1 << 13) / fileStep);

    if (rowsPerStrip < 1)
        rowsPerStrip = 1;

    if (rowsPerStrip > height)
        rowsPerStrip = height;

    // do NOT put "wb" as the mode, because the b means "big endian" mode, not "binary" mode.
    // http://www.remotesensing.org/libtiff/man/TIFFOpen.3tiff.html
    TIFF *tif;

    std::vector<uint8_t> output;
    TiffEncoderBufHelper buf_helper(&output);
    tif = buf_helper.open();
    if (!tif) {
        return std::vector<uint8_t>();
    }

    // defaults for now, maybe base them on params in the future
    int predictor = PREDICTOR_HORIZONTAL;
    int colorspace = channels > 1 ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK;
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bitsPerChannel);
    TIFFSetField(tif, TIFFTAG_COMPRESSION, compression);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, colorspace);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, channels);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_TILEWIDTH, 256);
    TIFFSetField(tif, TIFFTAG_TILELENGTH, 256);

    int tilesize = TIFFTileSize(tif);
    int tiles0 = TIFFComputeTile(tif, 0, 0, 0, 0);
    int tiles1 = TIFFComputeTile(tif, width - 1, height - 1, 0, 0);
    int ntiles = TIFFNumberOfTiles(tif);

    cv::AutoBuffer<uint8_t> _buffer(tilesize + 32);
    uint8_t *buffer = _buffer;
    if (!buffer) {
        TIFFClose(tif);
        return std::vector<uint8_t>();
    }

    for (int r = 0; r < height; r += 256) {
        int bottom = r + 256;
        bottom = std::min(height - 1, bottom);
        int sub_rows = bottom - r;
        for (int c = 0; c < width; c += 256) {
            int right = c + 256;
            right = std::min(width - 1, right);
            int sub_cols = right - c;
            cv::Rect rect(c, r, sub_cols, sub_rows);
            cv::Mat mat_rect = mat(rect).clone();

            switch (channels) {
            case 3:
                cv::cvtColor(mat_rect, mat_rect, cv::COLOR_BGR2RGB);
                break;
            case 4:
                cv::cvtColor(mat_rect, mat_rect, cv::COLOR_BGRA2RGBA);
                break;
            default:
                break;
            }

            if (sub_rows != 256 || sub_cols != 256) {
                cv::copyMakeBorder(mat_rect, mat_rect, 0, 256 - sub_rows, 0, 256 - sub_cols, cv::BORDER_REPLICATE);
            }
            uint32_t tile = TIFFComputeTile(tif, c, r, 0, 0);
            int size = mat_rect.rows * mat_rect.cols * bitsPerChannel / 8 * channels;
            int write_result = TIFFWriteEncodedTile(tif, tile, mat_rect.data, size);
            if (write_result < 0) {
                TIFFClose(tif);
                return std::vector<uint8_t>();
            }
        }
    }

    TIFFClose(tif);
    return output;
}
std::vector<uint8_t> tiff_encode_pyramid(const cv::Mat &mat, TiffCompression compression) {
    int channels = mat.channels();
    int depth = mat.depth();

    int bitsPerChannel = -1;
    switch (depth) {
    case CV_8U: {
        bitsPerChannel = 8;
        break;
    }
    case CV_16U: {
        bitsPerChannel = 16;
        break;
    }
    default: { return std::vector<uint8_t>(); }
    }
    const int bitsPerByte = 8;

    // do NOT put "wb" as the mode, because the b means "big endian" mode, not "binary" mode.
    // http://www.remotesensing.org/libtiff/man/TIFFOpen.3tiff.html
    TIFF *tif;

    std::vector<uint8_t> output;
    TiffEncoderBufHelper buf_helper(&output);
    tif = buf_helper.open();
    if (!tif) {
        return std::vector<uint8_t>();
    }

    cv::Mat page = mat;
    int npyr = 0;
    while (page.rows > 256 && page.cols > 256) {
        const toff_t nBaseDirOffset = TIFFCurrentDirOffset(tif);
        TIFFCreateDirectory(tif);

        int width = page.cols, height = page.rows;
        size_t fileStep = (width * channels * bitsPerChannel) / bitsPerByte;
        int rowsPerStrip = (int)((1 << 13) / fileStep);
        if (rowsPerStrip < 1)
            rowsPerStrip = 1;
        if (rowsPerStrip > height)
            rowsPerStrip = height;

        // defaults for now, maybe base them on params in the future
        int predictor = PREDICTOR_HORIZONTAL;
        int colorspace = channels > 1 ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK;

        if (npyr != 0) {
            TIFFSetField(tif, TIFFTAG_SUBFILETYPE, FILETYPE_REDUCEDIMAGE);
        }

        TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
        TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
        TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bitsPerChannel);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, compression);
        TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
        TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, colorspace);
        TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, channels);
        TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(tif, TIFFTAG_TILEWIDTH, 256);
        TIFFSetField(tif, TIFFTAG_TILELENGTH, 256);

        int tilesize = TIFFTileSize(tif);
        int tiles0 = TIFFComputeTile(tif, 0, 0, 0, 0);
        int tiles1 = TIFFComputeTile(tif, width - 1, height - 1, 0, 0);
        int ntiles = TIFFNumberOfTiles(tif);

        for (int r = 0; r < height; r += 256) {
            int bottom = r + 256;
            bottom = std::min(height - 1, bottom);
            int sub_rows = bottom - r;
            for (int c = 0; c < width; c += 256) {
                int right = c + 256;
                right = std::min(width - 1, right);
                int sub_cols = right - c;
                cv::Rect rect(c, r, sub_cols, sub_rows);
                cv::Mat mat_rect = page(rect).clone();

                switch (channels) {
                case 3:
                    cv::cvtColor(mat_rect, mat_rect, cv::COLOR_BGR2RGB);
                    break;
                case 4:
                    cv::cvtColor(mat_rect, mat_rect, cv::COLOR_BGRA2RGBA);
                    break;
                default:
                    break;
                }

                if (sub_rows != 256 || sub_cols != 256) {
                    cv::copyMakeBorder(mat_rect, mat_rect, 0, 256 - sub_rows, 0, 256 - sub_cols, cv::BORDER_REPLICATE);
                }
                uint32_t tile = TIFFComputeTile(tif, c, r, 0, 0);
                int size = mat_rect.rows * mat_rect.cols * bitsPerChannel / 8 * channels;
                int write_result = TIFFWriteEncodedTile(tif, tile, mat_rect.data, size);
                if (write_result < 0) {
                    TIFFClose(tif);
                    return std::vector<uint8_t>();
                }
            }
        }

        if (TIFFWriteCheck(tif, 1, "tiff_encode_pyramid") == 0) {
            TIFFClose(tif);
            return std::vector<uint8_t>();
        }

        TIFFWriteDirectory(tif);
        int ndir = TIFFNumberOfDirectories(tif);
        TIFFSetDirectory(tif, ndir - 1);
        const toff_t nOffset = TIFFCurrentDirOffset(tif);
        TIFFSetSubDirectory(tif, nBaseDirOffset);

        cv::pyrDown(page, page);
        npyr++;
    }

    TIFFClose(tif);
    return output;
}
} // namespace h2o
