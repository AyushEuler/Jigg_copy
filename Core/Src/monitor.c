bmsStatus_t readCellTemperatures(bmsMainData_t *mdata)
{
	bmsStatus_t status = BMS_OK;

	if((NULL == mdata))
	{
		status = BMS_CMU_FUNCTION_PARAMETER_ERROR;
		return status;
	}
	
	memset((void*)&mdata->cmuData.cmuTemperatures,0,sizeof(cmuTemperatureData_t));
	status = adBms6815_start_aux_voltage_conversion(mdata->cmuData.cmuCount, mdata->cmuData.cmuCellData);
	CHECK_STATUS(status);
	status = adBms6815_read_aux_voltages(mdata->cmuData.cmuCount, mdata->cmuData.cmuCellData);
	CHECK_STATUS(status);
	status = checkPackTemperatures(mdata);

	return status;
}