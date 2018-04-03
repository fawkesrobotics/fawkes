/****************************************************************************
 *  Clips -- Schema SlotValue
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** SlotValue representation for JSON transfer. */
export interface SlotValue
{
	name: string;
	type?: SlotValue.TypeEnum;
	is_multifield: boolean;
	values: Array<string>;
}

export namespace SlotValue
{
	export const API_VERSION: string = "v1beta1";

	export type TypeEnum = 'FLOAT' | 'INTEGER' | 'SYMBOL' | 'STRING' | 'EXTERNAL-ADDRESS';
	export const TypeEnum = {
		FLOAT: 'FLOAT' as TypeEnum,
		INTEGER: 'INTEGER' as TypeEnum,
		SYMBOL: 'SYMBOL' as TypeEnum,
		STRING: 'STRING' as TypeEnum,
		EXTERNAL_ADDRESS: 'EXTERNAL-ADDRESS' as TypeEnum
	}
}