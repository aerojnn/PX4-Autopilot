
#include "ParameterBase.hpp"
#include "ParameterServer.hpp"
#include "ParameterClient.hpp"

static ParameterBase *parameterImpl {nullptr};

#include "param.h"

void param_init()
{
	if (parameterImpl == nullptr) {
#ifdef CONFIG_PARAMETER_CLIENT
		parameterImpl = new ParameterClient();
#else
		parameterImpl = new ParameterServer();
#endif
	}
}

void param_notify_changes()
{
	if (parameterImpl) {
		parameterImpl->notifyChanges();
	}
}

param_t param_find(const char *name)
{
	if (parameterImpl) {
		return parameterImpl->findParameter(name, true);
	}

	return PARAM_INVALID;
}

param_t param_find_no_notification(const char *name)
{
	if (parameterImpl) {
		return parameterImpl->findParameter(name, false);
	}

	return PARAM_INVALID;
}

size_t param_size(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->getParameterSize(param);
	}

	return 0;
}

unsigned param_count()
{
	if (parameterImpl) {
		return parameterImpl->count();
	}

	return 0;
}

unsigned param_count_used()
{
	if (parameterImpl) {
		return parameterImpl->countUsed();
	}

	return 0;
}

param_t param_for_index(unsigned index)
{
	if (parameterImpl) {
		return parameterImpl->forIndex(index);
	}

	return PARAM_INVALID;
}

param_t param_for_used_index(unsigned index)
{
	if (parameterImpl) {
		return parameterImpl->forUsedIndex(index);
	}

	return PARAM_INVALID;
}

int param_get_index(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->getParameterIndex(param);
	}

	return -1;
}

int param_get_used_index(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->getParameterUsedIndex(param);
	}

	return -1;
}

const char *param_name(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->getParameterName(param);
	}

	return nullptr;
}

param_type_t param_type(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->getParameterType(param);
	}

	return PARAM_TYPE_UNKNOWN;
}

bool param_is_volatile(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->isParameterVolatile(param);
	}

	return false;
}


bool param_value_unsaved(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->isParameterValueUnsaved(param);
	}

	return true;
}

int param_get(param_t param, void *val)
{
	if (parameterImpl) {
		return parameterImpl->getParameterValue(param, val);
	}

	return -1;
}

int param_get_default_value(param_t param, void *default_val)
{
	if (parameterImpl) {
		parameterImpl->getParameterDefaultValue(param, default_val);
	}

	return -1;
}

int param_get_system_default_value(param_t param, void *default_val)
{
	if (parameterImpl) {
		parameterImpl->getParameterSystemDefaultValue(param, default_val);
	}

	return -1;
}

bool param_value_is_default(param_t param)
{
	if (parameterImpl) {
		parameterImpl->isParameterValueDefault(param);
	}

	return true;
}

void param_control_autosave(bool enable)
{
	if (parameterImpl) {
		parameterImpl->controlAutosave(enable);
	}
}

int param_set(param_t param, const void *val)
{
	if (parameterImpl) {
		return parameterImpl->setParameter(param, val, false, true);
	}

	return -1;
}

int param_set_no_notification(param_t param, const void *val)
{
	if (parameterImpl) {
		return parameterImpl->setParameter(param, val, false, false);
	}

	return -1;
}

bool param_used(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->isParameterUsed(param);
	}

	return false;
}

void param_set_used(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->setParameterUsed(param);
	}
}

int param_set_default_value(param_t param, const void *val)
{
	if (parameterImpl) {
		return parameterImpl->setParameterDefaultValue(param, val);
	}

	return -1;
}

int param_reset(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->resetParameter(param, true);
	}

	return -1;
}

int param_reset_no_notification(param_t param)
{
	if (parameterImpl) {
		return parameterImpl->resetParameter(param, false);
	}

	return -1;
}

void param_reset_all()
{
	if (parameterImpl) {
		return parameterImpl->resetAllParameters(true);
	}
}

void param_reset_excludes(const char *excludes[], int num_excludes)
{
	if (parameterImpl) {
		return parameterImpl->resetExcludes(excludes, num_excludes);
	}
}

void param_reset_specific(const char *resets[], int num_resets)
{
	if (parameterImpl) {
		return parameterImpl->resetSpecificParameter(resets, num_resets);
	}
}

int param_set_default_file(const char *filename)
{
	if (parameterImpl) {
		return parameterImpl->setDefaultFile(filename);
	}

	return -1;
}

const char *param_get_default_file()
{
	if (parameterImpl) {
		return parameterImpl->getDefaultFile();
	}

	return nullptr;
}

int param_set_backup_file(const char *filename)
{
	if (parameterImpl) {
		return parameterImpl->setBackupFile(filename);
	}

	return -1;
}

const char *param_get_backup_file()
{
	if (parameterImpl) {
		return parameterImpl->getBackupFile();
	}

	return nullptr;
}

int param_save_default()
{
	if (parameterImpl) {
		parameterImpl->autoSave(true);
		return 0;
	}

	return -1;
}

int param_load_default()
{
	if (parameterImpl) {
		return parameterImpl->loadDefault();
	}

	return -1;
}

int param_export(const char *filename, param_filter_func filter)
{
	if (parameterImpl) {
		return parameterImpl->exportToFile(filename, filter);
	}

	return -1;
}

int param_import(int fd)
{
	if (parameterImpl) {
		return parameterImpl->importFromFileDescriptor(fd);
	}

	return -1;
}

int param_load(int fd)
{
	if (parameterImpl) {
		return parameterImpl->loadFromFileDescriptor(fd);
	}

	return -1;
}

int param_dump(int fd)
{
	if (parameterImpl) {
		return parameterImpl->bsonDump(fd);
	}

	return -1;
}

void param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used)
{
	if (parameterImpl) {
		parameterImpl->forEachParameter(func, arg, only_changed, only_used);
	}
}

uint32_t param_hash_check()
{
	if (parameterImpl) {
		parameterImpl->hashCheck();
	}

	return 0;
}

void param_print_status()
{
	if (parameterImpl) {
		parameterImpl->printStatus();
	}
}
