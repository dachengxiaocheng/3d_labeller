#ifndef PCL_FILTERS_IMPL_PASSTHROUGH_HPP_
#define PCL_FILTERS_IMPL_PASSTHROUGH_HPP_

#include <pcl/filters/passthrough.h>
#include <pcl/common/io.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PassThrough<PointT>::applyFilter (PointCloud &output)
{
  std::vector<int> indices;
  if (keep_organized_)
  {
    bool temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    applyFilterIndices (indices);
    extract_removed_indices_ = temp;

    output = *input_;
    for (int rii = 0; rii < static_cast<int> (removed_indices_->size ()); ++rii)  // rii = removed indices iterator
      output.points[(*removed_indices_)[rii]].x = output.points[(*removed_indices_)[rii]].y = output.points[(*removed_indices_)[rii]].z = user_filter_value_;
    if (!pcl_isfinite (user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    applyFilterIndices (indices);
    copyPointCloud (*input_, indices, output);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PassThrough<PointT>::applyFilterIndices (std::vector<int> &indices)
{
  // The arrays to be used
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  // Has a field name been specified?
  if (filter_field_name_.empty ())
  {
    // Only filter for non-finite entries then
    for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)  // iii = input indices iterator
    {
      // Non-finite entries are always passed to removed indices
      if (!pcl_isfinite (input_->points[(*indices_)[iii]].x) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].y) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].z))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = (*indices_)[iii];
        continue;
      }
      indices[oii++] = (*indices_)[iii];
    }
  }
  else
  {
    // Attempt to get the field name's index
    std::vector<sensor_msgs::PointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
    {
      PCL_WARN ("[pcl::%s::applyFilter] Unable to find field name in point type.\n", getClassName ().c_str ());
      indices.clear ();
      removed_indices_->clear ();
      return;
    }

    // Filter for non-finite entries and the specified field limits
    for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)  // iii = input indices iterator
    {
      // Non-finite entries are always passed to removed indices
      if (!pcl_isfinite (input_->points[(*indices_)[iii]].x) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].y) ||
          !pcl_isfinite (input_->points[(*indices_)[iii]].z))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = (*indices_)[iii];
        continue;
      }

      // Get the field's value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[(*indices_)[iii]]);
      int field_value = 0;
      memcpy (&field_value, pt_data + fields[distance_idx].offset, sizeof (int));

      // Outside of the field limits are passed to removed indices
      if (!negative_ && (field_value < filter_limit_min_ || field_value > filter_limit_max_))
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = (*indices_)[iii];
        continue;
      }

      // Inside of the field limits are passed to removed indices if negative was set
      if (negative_ && field_value > filter_limit_min_ && field_value < filter_limit_max_)
      {
        if (extract_removed_indices_)
          (*removed_indices_)[rii++] = (*indices_)[iii];
        continue;
      }

      // Otherwise it was a normal point for output (inlier)
      indices[oii++] = (*indices_)[iii];
    }
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_PassThrough(T) template class PCL_EXPORTS pcl::PassThrough<T>;

#endif  // PCL_FILTERS_IMPL_PASSTHROUGH_HPP_

