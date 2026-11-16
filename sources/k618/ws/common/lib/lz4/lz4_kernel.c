// SPDX-License-Identifier: GPL-2.0-only
/*
 * The kernel's LZ4 API (include/linux/lz4.h) on upstream LZ4 v1.10.1 (lz4_up.c, through lz4k.h),
 * with the exports lib/lz4 had before. Made by tools/kernel-patches/lz4_upgrade.py.
 */
#include <linux/build_bug.h>
#include <linux/lz4.h>
#include <linux/module.h>
#include "lz4k.h"

static_assert(LZ4_MEM_COMPRESS == LZ4K_STREAM_SIZE && sizeof(LZ4_stream_t) == LZ4K_STREAM_SIZE);
static_assert(LZ4HC_MEM_COMPRESS == LZ4K_STREAMHC_SIZE &&
	      sizeof(LZ4_streamHC_t) == LZ4K_STREAMHC_SIZE);
static_assert(sizeof(LZ4_streamDecode_t) == LZ4K_STREAMDECODE_SIZE);
static_assert(__alignof__(LZ4_stream_t) >= 8 && __alignof__(LZ4_streamHC_t) >= 8 &&
	      __alignof__(LZ4_streamDecode_t) >= 8);

int LZ4_compress_default(const char *source, char *dest, int inputSize, int maxOutputSize,
			 void *wrkmem)
{
	return lz4k_compress_fast(wrkmem, source, dest, inputSize, maxOutputSize,
				  LZ4_ACCELERATION_DEFAULT);
}
EXPORT_SYMBOL(LZ4_compress_default);

int LZ4_compress_fast(const char *source, char *dest, int inputSize, int maxOutputSize,
		      int acceleration, void *wrkmem)
{
	return lz4k_compress_fast(wrkmem, source, dest, inputSize, maxOutputSize, acceleration);
}
EXPORT_SYMBOL(LZ4_compress_fast);

int LZ4_compress_destSize(const char *src, char *dst, int *srcSizePtr, int targetDstSize,
			  void *wrkmem)
{
	return lz4k_compress_destSize(wrkmem, src, dst, srcSizePtr, targetDstSize);
}
EXPORT_SYMBOL(LZ4_compress_destSize);

int LZ4_decompress_fast(const char *source, char *dest, int originalSize)
{
	return lz4k_decompress_fast(source, dest, originalSize);
}
EXPORT_SYMBOL(LZ4_decompress_fast);

int LZ4_decompress_safe(const char *source, char *dest, int compressedSize,
			int maxDecompressedSize)
{
	return lz4k_decompress_safe(source, dest, compressedSize, maxDecompressedSize);
}
EXPORT_SYMBOL(LZ4_decompress_safe);

int LZ4_decompress_safe_partial(const char *src, char *dst, int compressedSize,
				int targetOutputSize, int dstCapacity)
{
	return lz4k_decompress_safe_partial(src, dst, compressedSize, targetOutputSize,
					    dstCapacity);
}
EXPORT_SYMBOL(LZ4_decompress_safe_partial);

int LZ4_compress_HC(const char *src, char *dst, int srcSize, int maxDstSize,
		    int compressionLevel, void *wrkmem)
{
	return lz4k_compress_HC(wrkmem, src, dst, srcSize, maxDstSize, compressionLevel);
}
EXPORT_SYMBOL(LZ4_compress_HC);

void LZ4_resetStreamHC(LZ4_streamHC_t *LZ4_streamHCPtr, int compressionLevel)
{
	lz4k_resetStreamHC(LZ4_streamHCPtr, compressionLevel);
}
EXPORT_SYMBOL(LZ4_resetStreamHC);	/* k618: 6.18 exports it (zram's lz4hc backend) */

int LZ4_loadDictHC(LZ4_streamHC_t *LZ4_streamHCPtr, const char *dictionary, int dictSize)
{
	return lz4k_loadDictHC(LZ4_streamHCPtr, dictionary, dictSize);
}
EXPORT_SYMBOL(LZ4_loadDictHC);

int LZ4_compress_HC_continue(LZ4_streamHC_t *LZ4_streamHCPtr, const char *source, char *dest,
			     int inputSize, int maxOutputSize)
{
	return lz4k_compress_HC_continue(LZ4_streamHCPtr, source, dest, inputSize, maxOutputSize);
}
EXPORT_SYMBOL(LZ4_compress_HC_continue);

int LZ4_saveDictHC(LZ4_streamHC_t *LZ4_streamHCPtr, char *safeBuffer, int dictSize)
{
	return lz4k_saveDictHC(LZ4_streamHCPtr, safeBuffer, dictSize);
}
EXPORT_SYMBOL(LZ4_saveDictHC);

void LZ4_resetStream(LZ4_stream_t *LZ4_stream)
{
	lz4k_resetStream(LZ4_stream);
}

int LZ4_loadDict(LZ4_stream_t *LZ4_dict, const char *dictionary, int dictSize)
{
	return lz4k_loadDict(LZ4_dict, dictionary, dictSize);
}
EXPORT_SYMBOL(LZ4_loadDict);

int LZ4_saveDict(LZ4_stream_t *LZ4_dict, char *safeBuffer, int dictSize)
{
	return lz4k_saveDict(LZ4_dict, safeBuffer, dictSize);
}
EXPORT_SYMBOL(LZ4_saveDict);

int LZ4_compress_fast_continue(LZ4_stream_t *LZ4_stream, const char *source, char *dest, int inputSize,
			       int maxOutputSize, int acceleration)
{
	return lz4k_compress_fast_continue(LZ4_stream, source, dest, inputSize, maxOutputSize, acceleration);
}
EXPORT_SYMBOL(LZ4_compress_fast_continue);

int LZ4_setStreamDecode(LZ4_streamDecode_t *LZ4_streamDecode, const char *dictionary,
			int dictSize)
{
	return lz4k_setStreamDecode(LZ4_streamDecode, dictionary, dictSize);
}
EXPORT_SYMBOL(LZ4_setStreamDecode);

int LZ4_decompress_safe_continue(LZ4_streamDecode_t *LZ4_streamDecode, const char *source,
				 char *dest, int compressedSize, int maxOutputSize)
{
	return lz4k_decompress_safe_continue(LZ4_streamDecode, source, dest, compressedSize,
					     maxOutputSize);
}
EXPORT_SYMBOL(LZ4_decompress_safe_continue);

int LZ4_decompress_fast_continue(LZ4_streamDecode_t *LZ4_streamDecode, const char *source,
				 char *dest, int originalSize)
{
	return lz4k_decompress_fast_continue(LZ4_streamDecode, source, dest, originalSize);
}
EXPORT_SYMBOL(LZ4_decompress_fast_continue);

int LZ4_decompress_safe_usingDict(const char *source, char *dest, int compressedSize,
				  int maxOutputSize, const char *dictStart, int dictSize)
{
	return lz4k_decompress_safe_usingDict(source, dest, compressedSize, maxOutputSize,
					      dictStart, dictSize);
}
EXPORT_SYMBOL(LZ4_decompress_safe_usingDict);

int LZ4_decompress_fast_usingDict(const char *source, char *dest, int originalSize,
				  const char *dictStart, int dictSize)
{
	return lz4k_decompress_fast_usingDict(source, dest, originalSize, dictStart, dictSize);
}
EXPORT_SYMBOL(LZ4_decompress_fast_usingDict);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("LZ4 compression and decompression (upstream LZ4 v1.10.1)");
