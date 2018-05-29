/****************************************************************************
 *  Image -- Schema ImageInfo
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Image REST API.
 *  Access images through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** ImageInfo representation for JSON transfer. */
export interface ImageInfo
{
	kind: string;
	apiVersion: string;
	id: string;
	colorspace: string;
	frame: string;
	width: number;
	height: number;
	mem_size: number;
}

export namespace ImageInfo
{
	export const API_VERSION: string = "v1beta1";

}