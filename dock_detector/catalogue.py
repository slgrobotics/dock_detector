"""Named dock acquisition state, independent of ROS graph and geometry fitting."""
class Selection:
    def __init__(self, docks):
        self.docks = docks
        self.dock_id = ''
        self.acquisition_id = ''

    def select(self, activate, dock_id, acquisition_id):
        if not acquisition_id:
            return False, 'acquisition_id must not be empty'
        if activate:
            if dock_id not in self.docks:
                return False, 'unknown dock_id'
            self.dock_id, self.acquisition_id = dock_id, acquisition_id
            return True, 'selected'
        if (dock_id, acquisition_id) != (self.dock_id, self.acquisition_id):
            return False, 'release does not match active acquisition'
        self.dock_id = self.acquisition_id = ''
        return True, 'released'
