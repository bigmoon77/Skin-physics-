struct AnimationSkinRecord
{
    uint mesh_node_channel_offset;
    uint skin_index;
    uint first_joint_record;
    uint joint_count;
};

struct AnimationSkinJointRecord
{
    uint joint_channel_offset;
    uint inverse_bind_record_index;
    uint joint_node_index;
    uint reserved;
};

StructuredBuffer<AnimationSkinRecord> skin_records : register(t0);
StructuredBuffer<AnimationSkinJointRecord> skin_joint_records : register(t1);
RWStructuredBuffer<AnimationSkinRecord> skin_records_out : register(u0);
RWStructuredBuffer<AnimationSkinJointRecord> skin_joint_records_out : register(u1);

[numthreads(1, 1, 1)]
void copy_animation_skin_abi(uint3 dispatch_id : SV_DispatchThreadID)
{
    skin_records_out[0] = skin_records[0];
    skin_joint_records_out[0] = skin_joint_records[0];
}
